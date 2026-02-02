#!/usr/bin/env python3
"""
Target Navigation GUI for RZ/V2N
================================

GTK3-based GUI that displays:
    - Left panel: Cartographer SLAM map with robot position
    - Right panel: Camera feed with YOLO detection overlays
    - Status bar: Detection info, distance, state

Works with Wayland on RZ/V2N embedded display.

Usage:
    export XDG_RUNTIME_DIR=/run/user/996
    export WAYLAND_DISPLAY=wayland-1
    ros2 run bowling_target_nav target_gui_node

Subscriptions:
    /map (nav_msgs/OccupancyGrid) - Cartographer SLAM map
    /scan (sensor_msgs/LaserScan) - LiDAR data
    /odom (nav_msgs/Odometry) - Robot odometry
    /target_pose (geometry_msgs/PoseStamped) - Detected target
    /target_detection (std_msgs/String) - Detection JSON

Press Q or ESC to quit.
"""

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib

import cv2
import numpy as np
import math
import threading
import time
import json
import os
import cairo

# Disable OpenCL for RZ/V2N
cv2.ocl.setUseOpenCL(False)

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry, OccupancyGrid
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("WARNING: ROS2 not available")

# Import detector
try:
    from bowling_target_nav.detectors import YoloDetector
    HAS_DETECTOR = True
except ImportError:
    HAS_DETECTOR = False
    print("WARNING: YoloDetector not available")


# =============================================================================
# Global State (thread-safe)
# =============================================================================
lock = threading.Lock()
cam_lock = threading.Lock()
map_lock = threading.Lock()
running = True

# Robot state
robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0

# Target state
target_x, target_y = 0.0, 0.0
target_distance = 0.0
target_angle = 0.0
target_class = ""
target_confidence = 0.0
has_target = False

# Map data
carto_map = None  # (width, height, resolution, origin_x, origin_y)
carto_map_image = None  # RGBA numpy array

# Camera data
camera_frame = None  # (frame_rgba, width, height)
detection_count = 0
cam_fps = 0.0

# Follower state
follower_state = "IDLE"


def quat_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# =============================================================================
# Camera + Detection Thread
# =============================================================================
def camera_thread():
    """Camera capture with YOLO detection."""
    global camera_frame, detection_count, cam_fps, running
    global has_target

    # Find model
    model_paths = [
        '/opt/ros/humble/share/bowling_target_nav/models/bowling_yolov5.onnx',
        '/tmp/bowling_yolov5.onnx',
        os.path.join(os.path.dirname(__file__), '..', '..', 'models', 'bowling_yolov5.onnx'),
    ]

    detector = None
    if HAS_DETECTOR:
        for path in model_paths:
            if os.path.exists(path):
                try:
                    detector = YoloDetector(path, conf_threshold=0.3)
                    print(f"Loaded model: {path}", flush=True)
                    break
                except Exception as e:
                    print(f"Model load error: {e}", flush=True)

    # Open camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 15)

    if not cap.isOpened():
        print("Cannot open camera", flush=True)
        return

    print(f"Camera: {int(cap.get(3))}x{int(cap.get(4))}", flush=True)

    frame_count = 0
    fps_time = time.time()
    detect_every = 3

    while running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        frame_count += 1
        dets = []

        # Run detection
        if detector and frame_count % detect_every == 0:
            try:
                all_dets = detector.detect(frame)
                # Filter for Pins
                dets = [d for d in all_dets if d.class_name == "Pins"]
                detection_count = len(dets)
                has_target = len(dets) > 0
            except Exception as e:
                pass

        # Draw detections
        if detector and dets:
            frame = detector.draw_detections(frame, dets)

            # Draw target indicator
            best = detector.get_largest(dets)
            if best:
                cx, cy = int(best.center_x), int(best.center_y)
                cv2.circle(frame, (cx, cy), 8, (0, 255, 255), 2)
                cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 255, 255), 1)
                cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 255, 255), 1)

        # FPS calculation
        if frame_count % 10 == 0:
            cam_fps = 10.0 / (time.time() - fps_time)
            fps_time = time.time()

        # Status overlay
        status = f"FPS:{cam_fps:.0f} Det:{detection_count}"
        if has_target:
            status += f" TRACKING"
        cv2.putText(frame, status, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # Convert to RGBA
        frame_rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        h, w = frame_rgba.shape[:2]

        with cam_lock:
            camera_frame = (np.ascontiguousarray(frame_rgba), w, h)

        time.sleep(0.02)

    cap.release()


# =============================================================================
# ROS2 Node
# =============================================================================
if HAS_ROS:
    class GuiNode(Node):
        """ROS2 node for GUI subscriptions."""

        def __init__(self):
            super().__init__('target_gui_node')

            self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
            self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
            self.create_subscription(PoseStamped, '/target_pose', self.target_cb, 10)
            self.create_subscription(String, '/target_detection', self.detection_cb, 10)

        def odom_cb(self, msg):
            global robot_x, robot_y, robot_theta
            with lock:
                robot_x = msg.pose.pose.position.x
                robot_y = msg.pose.pose.position.y
                q = msg.pose.pose.orientation
                robot_theta = quat_to_yaw(q.x, q.y, q.z, q.w)

        def target_cb(self, msg):
            global target_x, target_y
            with lock:
                target_x = msg.pose.position.x
                target_y = msg.pose.position.y

        def detection_cb(self, msg):
            global target_distance, target_angle, target_class, target_confidence
            try:
                data = json.loads(msg.data)
                with lock:
                    target_class = data.get('class_name', '')
                    target_confidence = data.get('confidence', 0)
                    est = data.get('estimated', {})
                    target_distance = est.get('distance_m', 0)
                    target_angle = est.get('angle_deg', 0)
            except:
                pass

        def map_cb(self, msg):
            global carto_map, carto_map_image

            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y

            data = np.array(msg.data, dtype=np.int8).reshape((height, width))

            # Create RGBA image
            img = np.zeros((height, width, 4), dtype=np.uint8)
            img[:, :, 3] = 255

            # Unknown: gray
            unknown = data == -1
            img[unknown, 0] = 50
            img[unknown, 1] = 50
            img[unknown, 2] = 50

            # Free: dark
            free = data == 0
            img[free, 0] = 20
            img[free, 1] = 20
            img[free, 2] = 20

            # Occupied: green
            occupied = data > 50
            img[occupied, 0] = 0
            img[occupied, 1] = 255
            img[occupied, 2] = 0

            # Partial: dim green
            partial = (data > 0) & (data <= 50)
            img[partial, 0] = 0
            img[partial, 1] = 128
            img[partial, 2] = 0

            img = np.flipud(img)

            with map_lock:
                carto_map = (width, height, resolution, origin_x, origin_y)
                carto_map_image = np.ascontiguousarray(img)

    def ros_thread():
        """ROS2 spin thread."""
        global running
        try:
            rclpy.init()
            node = GuiNode()
            while running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"ROS error: {e}", flush=True)


# =============================================================================
# GTK3 GUI
# =============================================================================
class TargetNavGUI(Gtk.Window):
    """Fullscreen GTK3 GUI with map and camera."""

    def __init__(self):
        super().__init__(title="Bowling Target Navigation")
        self.fullscreen()

        self.da = Gtk.DrawingArea()
        self.da.connect('draw', self.on_draw)
        self.add(self.da)

        self.connect('key-press-event', self.on_key)
        self.connect('destroy', self.on_quit)

        GLib.timeout_add(80, self.on_tick)

    def on_key(self, widget, event):
        key = Gdk.keyval_name(event.keyval).lower()
        if key in ('q', 'escape'):
            self.on_quit(None)
        return True

    def on_tick(self):
        self.da.queue_draw()
        return True

    def on_draw(self, widget, cr):
        alloc = widget.get_allocation()
        W, H = alloc.width, alloc.height

        # Background
        cr.set_source_rgb(0.05, 0.05, 0.05)
        cr.paint()

        half_w = W // 2
        margin = 8

        # === LEFT: Map ===
        map_size = min(H - 80, half_w - 15)
        map_x, map_y = margin, 50

        cr.set_source_rgb(0, 0.9, 0)
        cr.set_font_size(16)
        cr.move_to(map_x, 30)
        cr.show_text("SLAM Map")

        # Map background
        cr.set_source_rgb(0.03, 0.03, 0.03)
        cr.rectangle(map_x, map_y, map_size, map_size)
        cr.fill()

        with map_lock:
            map_info = carto_map
            map_img = carto_map_image.copy() if carto_map_image is not None else None

        if map_img is not None and map_info is not None:
            width, height, resolution, origin_x, origin_y = map_info

            # Draw robot on map
            with lock:
                rx, ry, rt = robot_x, robot_y, robot_theta
                tx, ty = target_x, target_y

            # Robot pixel position
            rpx = int((rx - origin_x) / resolution)
            rpy = int((ry - origin_y) / resolution)
            rpy = height - rpy - 1

            if 0 <= rpx < width and 0 <= rpy < height:
                cv2.circle(map_img, (rpx, rpy), 5, (0, 128, 255, 255), -1)
                arrow_len = 12
                dx = int(arrow_len * math.cos(rt))
                dy = int(-arrow_len * math.sin(rt))
                cv2.arrowedLine(map_img, (rpx, rpy), (rpx + dx, rpy + dy), (255, 255, 0, 255), 2)

            # Target position (if valid)
            if has_target and (tx != 0 or ty != 0):
                tpx = int((tx - origin_x) / resolution)
                tpy = int((ty - origin_y) / resolution)
                tpy = height - tpy - 1
                if 0 <= tpx < width and 0 <= tpy < height:
                    cv2.circle(map_img, (tpx, tpy), 6, (255, 0, 0, 255), 2)
                    cv2.drawMarker(map_img, (tpx, tpy), (255, 0, 0, 255),
                                   cv2.MARKER_CROSS, 10, 2)

            # Scale to fit
            scale = min(map_size / width, map_size / height)
            new_w, new_h = int(width * scale), int(height * scale)

            if new_w > 0 and new_h > 0:
                scaled = cv2.resize(map_img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
                scaled = np.ascontiguousarray(scaled)

                offset_x = map_x + (map_size - new_w) // 2
                offset_y = map_y + (map_size - new_h) // 2

                surface = cairo.ImageSurface.create_for_data(
                    scaled, cairo.FORMAT_ARGB32, new_w, new_h, new_w * 4
                )
                cr.set_source_surface(surface, offset_x, offset_y)
                cr.paint()

                # Info
                cr.set_source_rgb(0.5, 0.5, 0.5)
                cr.set_font_size(10)
                cr.move_to(map_x, map_y + map_size + 15)
                cr.show_text(f"Robot:({rx:.2f},{ry:.2f}) Map:{width}x{height}")
        else:
            cr.set_source_rgb(0.3, 0.3, 0.3)
            cr.set_font_size(14)
            cr.move_to(map_x + 30, map_y + map_size // 2)
            cr.show_text("Waiting for /map...")

        # === RIGHT: Camera ===
        cam_x = half_w + margin
        cam_w = half_w - margin * 2
        cam_h = H - 120
        cam_y = 50

        cr.set_source_rgb(0, 0.7, 1)
        cr.set_font_size(16)
        cr.move_to(cam_x, 30)
        cr.show_text("Camera + Detection")

        with cam_lock:
            cam_data = camera_frame

        if cam_data:
            frame_arr, fw, fh = cam_data
            sf = min(cam_w / fw, cam_h / fh)
            new_w, new_h = int(fw * sf), int(fh * sf)

            resized = cv2.resize(frame_arr, (new_w, new_h))
            resized = np.ascontiguousarray(resized)

            offset_x = cam_x + (cam_w - new_w) // 2
            offset_y = cam_y + (cam_h - new_h) // 2

            surface = cairo.ImageSurface.create_for_data(
                resized, cairo.FORMAT_ARGB32, new_w, new_h, new_w * 4
            )
            cr.set_source_surface(surface, offset_x, offset_y)
            cr.paint()
        else:
            cr.set_source_rgb(0.15, 0.15, 0.15)
            cr.rectangle(cam_x, cam_y, cam_w, cam_h)
            cr.fill()
            cr.set_source_rgb(0.4, 0.4, 0.4)
            cr.move_to(cam_x + cam_w // 2 - 40, cam_y + cam_h // 2)
            cr.show_text("No Camera")

        # === BOTTOM: Status bar ===
        status_y = H - 35

        with lock:
            dist = target_distance
            ang = target_angle
            cls = target_class
            conf = target_confidence

        # Status background
        cr.set_source_rgb(0.1, 0.1, 0.1)
        cr.rectangle(0, status_y - 5, W, 40)
        cr.fill()

        # Status text
        if has_target:
            cr.set_source_rgb(0, 1, 0)
            status = f"TARGET: {cls} | Distance: {dist:.2f}m | Angle: {ang:.1f}deg | Conf: {conf:.0%}"
        else:
            cr.set_source_rgb(0.5, 0.5, 0.5)
            status = "No target detected - searching..."

        cr.set_font_size(14)
        cr.move_to(margin, status_y + 12)
        cr.show_text(status)

        # Help
        cr.set_source_rgb(0.3, 0.3, 0.3)
        cr.set_font_size(10)
        cr.move_to(W - 60, status_y + 12)
        cr.show_text("Q=Quit")

        return False

    def on_quit(self, widget):
        global running
        running = False
        Gtk.main_quit()


# =============================================================================
# Main
# =============================================================================
def main():
    global running

    print(f"Starting GUI... HAS_ROS={HAS_ROS}", flush=True)

    # Start camera thread
    cam_thread = threading.Thread(target=camera_thread, daemon=True)
    cam_thread.start()

    # Start ROS thread
    if HAS_ROS:
        ros_t = threading.Thread(target=ros_thread, daemon=True)
        ros_t.start()

    # Create and run GUI
    win = TargetNavGUI()
    win.show_all()
    print("Bowling Target Navigation GUI (Q=Quit)", flush=True)

    try:
        Gtk.main()
    except KeyboardInterrupt:
        running = False


if __name__ == '__main__':
    main()
