#!/usr/bin/env python3
"""
Combined SLAM + Camera GUI with YOLO Detection - GTK3 Fullscreen
=================================================================

Thread-safe implementation with deadlock prevention:
- Uses threading.Event for shutdown signaling (atomic)
- Uses threading.RLock (reentrant lock) to prevent self-deadlock
- Lock acquisition with timeout to prevent deadlock
- Proper resource cleanup with context managers
- Double buffering to minimize lock hold time

Architecture:
- Main thread: GTK event loop + rendering
- Thread 1: ROS2 node (map, scan, TF)
- Thread 2: Camera capture + YOLO detection

Usage:
    ros2 run bowling_target_nav slam_camera_gui
"""

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib, GdkPixbuf

import math
import threading
import time
import cv2
import numpy as np
import os
import signal
import sys
import atexit
from contextlib import contextmanager

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf2_ros import TransformException

# YOLO Detector
from bowling_target_nav.detectors import YoloDetector
from bowling_target_nav.utils import DistanceEstimator


# ============================================================================
# Thread-safe shared state with deadlock prevention
# ============================================================================

class ThreadSafeState:
    """
    Thread-safe container for shared state.

    Deadlock prevention measures:
    - RLock (reentrant) allows same thread to acquire multiple times
    - Lock timeout prevents infinite waiting
    - Minimal lock hold time with copy-on-read
    - Event for shutdown (atomic, no lock needed)
    """

    LOCK_TIMEOUT = 0.1  # 100ms max wait for lock

    def __init__(self):
        # Use RLock (reentrant) - safer than Lock
        self._lock = threading.RLock()

        # Use Event for shutdown - atomic, no lock needed
        self._shutdown_event = threading.Event()

        # Map data
        self._map_img = None
        self._map_info = None
        self._map_count = 0

        # Robot pose
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_theta = 0.0

        # LiDAR data
        self._laser_points = np.empty((0, 2), dtype=np.float32)
        self._scan_count = 0

        # Camera data
        self._camera_frame = None
        self._detection_info = "Initializing..."

        # Error tracking
        self._errors = []

    @contextmanager
    def _acquire_lock(self, timeout=None):
        """
        Context manager for safe lock acquisition with timeout.
        Prevents deadlock by timing out if lock can't be acquired.
        """
        timeout = timeout or self.LOCK_TIMEOUT
        acquired = self._lock.acquire(timeout=timeout)
        if not acquired:
            raise TimeoutError(f"Could not acquire lock within {timeout}s")
        try:
            yield
        finally:
            self._lock.release()

    def _try_lock(self, timeout=None):
        """Try to acquire lock, return False if timeout."""
        timeout = timeout or self.LOCK_TIMEOUT
        return self._lock.acquire(timeout=timeout)

    def _release_lock(self):
        """Release lock safely."""
        try:
            self._lock.release()
        except RuntimeError:
            pass  # Lock not held

    @property
    def running(self):
        """Check if system should keep running (atomic, no lock)."""
        return not self._shutdown_event.is_set()

    def request_shutdown(self):
        """Request shutdown (atomic, no lock)."""
        self._shutdown_event.set()

    def wait_for_shutdown(self, timeout=None):
        """Wait for shutdown signal."""
        return self._shutdown_event.wait(timeout)

    def set_map(self, img, info):
        """Set map data with timeout protection."""
        if not self._try_lock():
            return False  # Skip update if can't get lock
        try:
            self._map_img = img
            self._map_info = info
            self._map_count += 1
            return True
        finally:
            self._release_lock()

    def get_map(self):
        """Get map data copy (minimizes lock hold time)."""
        if not self._try_lock():
            return None, None, 0
        try:
            if self._map_img is not None:
                return self._map_img.copy(), self._map_info, self._map_count
            return None, None, 0
        finally:
            self._release_lock()

    def set_robot_pose(self, x, y, theta):
        """Set robot pose with timeout protection."""
        if not self._try_lock():
            return False
        try:
            self._robot_x = x
            self._robot_y = y
            self._robot_theta = theta
            return True
        finally:
            self._release_lock()

    def get_robot_pose(self):
        """Get robot pose."""
        if not self._try_lock():
            return 0.0, 0.0, 0.0
        try:
            return self._robot_x, self._robot_y, self._robot_theta
        finally:
            self._release_lock()

    def set_laser(self, points):
        """Set laser points with timeout protection."""
        if not self._try_lock():
            return False
        try:
            self._laser_points = points
            self._scan_count += 1
            return True
        finally:
            self._release_lock()

    def get_laser(self):
        """Get laser points copy."""
        if not self._try_lock():
            return np.empty((0, 2), dtype=np.float32), 0, 0.0
        try:
            return self._laser_points.copy(), self._scan_count, 0.0
        finally:
            self._release_lock()

    def set_camera(self, frame, info):
        """Set camera frame with timeout protection."""
        if not self._try_lock():
            return False
        try:
            self._camera_frame = frame
            self._detection_info = info
            return True
        finally:
            self._release_lock()

    def get_camera(self):
        """Get camera frame copy."""
        if not self._try_lock():
            return None, "Lock timeout"
        try:
            if self._camera_frame is not None:
                return self._camera_frame.copy(), self._detection_info
            return None, self._detection_info
        finally:
            self._release_lock()

    def add_error(self, source, error):
        """Add error message (thread-safe)."""
        if not self._try_lock():
            return
        try:
            self._errors.append(f"{source}: {error}")
            # Keep only last 10 errors
            if len(self._errors) > 10:
                self._errors = self._errors[-10:]
        finally:
            self._release_lock()

    def get_errors(self):
        """Get error list copy."""
        if not self._try_lock():
            return []
        try:
            return self._errors[:]
        finally:
            self._release_lock()


# Global state instance
state = ThreadSafeState()

# Colors (BGR for OpenCV)
COLOR_FREE = (50, 50, 50)
COLOR_OCCUPIED = (0, 200, 255)  # Orange
COLOR_UNKNOWN = (30, 30, 30)
COLOR_ROBOT = (0, 255, 0)  # Green
COLOR_LASER = (0, 0, 255)  # Red
COLOR_GRID = (60, 60, 60)


def find_model_path():
    """Find YOLO model in standard locations."""
    paths = [
        os.path.expanduser('~/ros2_ws/install/bowling_target_nav/share/bowling_target_nav/models/bowling_yolov5.onnx'),
        os.path.expanduser('~/ros2_ws/src/bowling_target_nav/models/bowling_yolov5.onnx'),
        '/opt/ros/humble/share/bowling_target_nav/models/bowling_yolov5.onnx',
    ]
    for p in paths:
        if os.path.exists(p):
            return p
    return None


# ============================================================================
# ROS2 Node Thread
# ============================================================================

class SlamCameraNode(Node):
    """ROS2 node for map, scan, and TF - runs in separate thread."""

    def __init__(self):
        super().__init__('slam_camera_gui')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Use best effort QoS for sensor data (faster, less reliable)
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        # Map uses TRANSIENT_LOCAL - must match for late-joining subscribers
        from rclpy.qos import DurabilityPolicy
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, sensor_qos)

        self.get_logger().info('SLAM Camera GUI ROS node started')

    def map_cb(self, msg):
        """Process occupancy grid map."""
        if not state.running:
            return
        try:
            width = msg.info.width
            height = msg.info.height

            data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            img = np.zeros((height, width, 3), dtype=np.uint8)

            img[data == -1] = COLOR_UNKNOWN
            img[data == 0] = COLOR_FREE
            img[data > 0] = COLOR_OCCUPIED

            state.set_map(cv2.flip(img, 0), msg.info)
        except Exception as e:
            state.add_error('ros_map', str(e))

    def scan_cb(self, msg):
        """Process laser scan and update robot pose from TF."""
        if not state.running:
            return
        try:
            # Convert scan to points (vectorized with numpy)
            ranges = np.array(msg.ranges, dtype=np.float32)
            n = len(ranges)
            angles = np.linspace(
                msg.angle_min,
                msg.angle_min + (n - 1) * msg.angle_increment,
                n, dtype=np.float32)
            valid = (ranges > msg.range_min) & (ranges < msg.range_max)
            r_v = ranges[valid]
            a_v = angles[valid]
            if len(r_v) > 0:
                points = np.column_stack((r_v * np.cos(a_v), r_v * np.sin(a_v)))
            else:
                points = np.empty((0, 2), dtype=np.float32)
            state.set_laser(points)

            # Get robot pose from TF
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                q = transform.transform.rotation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                theta = math.atan2(siny_cosp, cosy_cosp)
                state.set_robot_pose(x, y, theta)
            except TransformException:
                pass  # TF not ready yet

        except Exception as e:
            state.add_error('ros_scan', str(e))


def ros_thread():
    """ROS2 spin thread with proper cleanup."""
    node = None
    print("[ROS Thread] Starting...", flush=True)

    try:
        rclpy.init()
        node = SlamCameraNode()

        while state.running:
            try:
                rclpy.spin_once(node, timeout_sec=0.02)  # 50Hz max
            except Exception as e:
                if state.running:  # Only log if not shutting down
                    state.add_error('ros_spin', str(e))
                break

    except Exception as e:
        state.add_error('ros_thread', str(e))
    finally:
        print("[ROS Thread] Cleaning up...", flush=True)
        if node:
            try:
                node.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass
        print("[ROS Thread] Stopped", flush=True)


# ============================================================================
# Camera + Detection Thread
# ============================================================================

def camera_thread():
    """Camera capture with YOLO detection - with proper cleanup."""
    cap = None
    detector = None
    print("[Camera Thread] Starting...", flush=True)

    try:
        # Find and load YOLO model
        model_path = find_model_path()
        estimator = None

        if model_path:
            print(f"[Camera Thread] Loading YOLO model: {model_path}", flush=True)
            try:
                detector = YoloDetector(model_path=model_path, conf_threshold=0.5)
                estimator = DistanceEstimator(
                    reference_box_height=100.0,
                    reference_distance=1.0,
                    frame_width=640,
                    frame_height=480,
                    horizontal_fov=60.0
                )
                print("[Camera Thread] YOLO detector loaded successfully", flush=True)
            except Exception as e:
                print(f"[Camera Thread] Failed to load YOLO: {e}", flush=True)
                state.add_error('yolo_load', str(e))
                detector = None
        else:
            print("[Camera Thread] YOLO model not found", flush=True)
            state.add_error('yolo', "Model not found")

        # Open camera with retries - use V4L2 backend directly (GStreamer fails on V2N)
        for attempt in range(3):
            if not state.running:
                return
            # Force V4L2 backend - GStreamer has issues on embedded devices
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if cap.isOpened():
                print(f"[Camera Thread] Camera opened on attempt {attempt + 1} (V4L2)", flush=True)
                break
            print(f"[Camera Thread] Camera open attempt {attempt + 1} failed", flush=True)
            time.sleep(1)

        if not cap or not cap.isOpened():
            state.add_error('camera', "Could not open camera")
            state.set_camera(None, "Camera not available")
            return

        # Configure camera for low latency
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        target_class = "Pins"
        frame_count = 0
        detect_count = 0
        last_fps_time = time.time()
        fps = 0.0

        # Detection cache - run detection every N frames for performance
        DETECT_EVERY_N = 3
        cached_detections = []
        cached_info = "No detection"

        while state.running:
            ret, frame = cap.read()
            if not ret:
                if state.running:
                    time.sleep(0.05)
                continue

            frame_count += 1
            detect_count += 1
            display_frame = frame.copy()
            info = cached_info

            # Run YOLO detection every N frames (performance optimization)
            run_detection = (detect_count >= DETECT_EVERY_N) and detector
            if run_detection:
                detect_count = 0
                try:
                    cached_detections = detector.detect(frame)
                    cached_info = "No detection"

                    for det in cached_detections:
                        if det.class_name == target_class and estimator:
                            distance, angle = estimator.estimate(det)
                            cached_info = f"{det.class_name}: {distance:.2f}m, {math.degrees(angle):.1f}deg"
                            break

                    if cached_detections and cached_info == "No detection":
                        cached_info = f"Found {len(cached_detections)} objects"

                    info = cached_info

                except Exception as e:
                    cached_info = "Detection error"
                    info = cached_info
                    state.add_error('detection', str(e))

            # Draw cached detections on every frame (smooth display)
            try:
                for det in cached_detections:
                    cls_name = det.class_name
                    conf = det.confidence
                    x1, y1, x2, y2 = det.x1, det.y1, det.x2, det.y2

                    color = (0, 255, 0) if cls_name == target_class else (0, 165, 255)

                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)

                    label = f"{cls_name}: {conf:.2f}"
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.rectangle(display_frame, (x1, y1 - th - 8), (x1 + tw + 4, y1), color, -1)
                    cv2.putText(display_frame, label, (x1 + 2, y1 - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

                    if cls_name == target_class and estimator:
                        distance, _ = estimator.estimate(det)
                        cv2.putText(display_frame, f"Dist: {distance:.2f}m",
                                    (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            except Exception as draw_err:
                print(f"[Camera Thread] Draw error: {draw_err}", flush=True)

            # Calculate FPS
            now = time.time()
            if now - last_fps_time >= 1.0:
                fps = frame_count / (now - last_fps_time)
                frame_count = 0
                last_fps_time = now

            cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Update state (with timeout protection)
            state.set_camera(cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB), info)

            # Minimal delay - camera buffer controls rate
            time.sleep(0.01)  # 10ms to prevent CPU spin

        print(f"[Camera Thread] Loop exited - running={state.running}", flush=True)

    except Exception as e:
        import traceback
        print(f"[Camera Thread] ERROR: {e}", flush=True)
        traceback.print_exc()
        state.add_error('camera_thread', str(e))
    finally:
        print("[Camera Thread] Cleaning up...", flush=True)
        if cap:
            try:
                cap.release()
            except:
                pass
        if detector:
            try:
                detector.cleanup()
            except:
                pass
        print("[Camera Thread] Stopped", flush=True)


# ============================================================================
# GTK GUI (Main Thread)
# ============================================================================

class SlamCameraGUI(Gtk.Window):
    """GTK3 fullscreen GUI - runs in main thread."""

    def __init__(self):
        super().__init__(title="SLAM + Camera + Detection")
        self.fullscreen()

        self.da = Gtk.DrawingArea()
        self.da.connect('draw', self.on_draw)
        self.add(self.da)

        self.connect('key-press-event', self.on_key)
        self.connect('destroy', self.on_quit)

        # Update timer - stores ID for cleanup (33ms = ~30 FPS)
        self._timer_id = GLib.timeout_add(33, self.on_tick)

        print("[GUI] Initialized", flush=True)

    def on_tick(self):
        if not state.running:
            self.on_quit(None)
            return False  # Stop timer
        self.da.queue_draw()
        return True  # Continue timer

    def on_key(self, widget, event):
        key = Gdk.keyval_name(event.keyval).lower()
        if key in ('q', 'escape'):
            self.on_quit(None)
        return True

    def on_draw(self, widget, cr):
        try:
            alloc = widget.get_allocation()
            W, H = alloc.width, alloc.height

            # Background
            cr.set_source_rgb(0.08, 0.08, 0.08)
            cr.paint()

            # Layout
            margin = 10
            title_h = 45
            panel_w = (W - margin * 3) // 2
            panel_h = H - title_h - margin * 2

            map_x, map_y = margin, title_h + margin
            cam_x, cam_y = margin * 2 + panel_w, title_h + margin

            # Title
            cr.set_source_rgb(0, 1, 0)
            cr.set_font_size(28)
            cr.move_to(margin, 35)
            cr.show_text("SLAM + Camera Viewer")

            # Status
            cr.set_font_size(14)
            cr.set_source_rgb(0.6, 0.6, 0.6)
            cr.move_to(W - 180, 30)
            cr.show_text("Press Q/ESC to quit")

            # Draw panels
            self.draw_map_panel(cr, map_x, map_y, panel_w, panel_h)
            self.draw_camera_panel(cr, cam_x, cam_y, panel_w, panel_h)

        except Exception as e:
            print(f"[GUI] Draw error: {e}", flush=True)
            import traceback
            traceback.print_exc()

        return False

    def draw_map_panel(self, cr, x, y, w, h):
        """Draw professional map panel."""
        # Panel background
        cr.set_source_rgb(0.05, 0.05, 0.05)
        cr.rectangle(x, y, w, h)
        cr.fill()

        # Border
        cr.set_source_rgb(0, 0.6, 0)
        cr.set_line_width(2)
        cr.rectangle(x, y, w, h)
        cr.stroke()

        # Header
        cr.set_source_rgb(0, 1, 0)
        cr.set_font_size(16)
        cr.move_to(x + 10, y + 22)
        cr.show_text("SLAM Map")

        # Get data from state (with timeout protection)
        local_map, info, mc = state.get_map()
        rx, ry, rt = state.get_robot_pose()
        lpoints, sc, _ = state.get_laser()

        content_y = y + 35
        content_h = h - 70

        if local_map is not None and info is not None:
            mh, mw = local_map.shape[:2]
            res = info.resolution
            ox = info.origin.position.x
            oy = info.origin.position.y

            scale = min((w - 20) / mw, content_h / mh)
            disp_w = int(mw * scale)
            disp_h = int(mh * scale)

            disp_x = x + (w - disp_w) // 2
            disp_y = content_y + (content_h - disp_h) // 2

            disp_map = cv2.resize(local_map, (disp_w, disp_h), interpolation=cv2.INTER_NEAREST)

            # Draw grid
            ppm = int(1.0 / res * scale)
            if ppm > 15:
                for gx in range(0, disp_w, ppm):
                    cv2.line(disp_map, (gx, 0), (gx, disp_h), COLOR_GRID, 1)
                for gy in range(0, disp_h, ppm):
                    cv2.line(disp_map, (0, gy), (disp_w, gy), COLOR_GRID, 1)

            # Draw laser points (vectorized)
            if len(lpoints) > 0:
                lp = np.asarray(lpoints, dtype=np.float32)
                cos_t, sin_t = math.cos(rt), math.sin(rt)
                wx = rx + lp[:, 0] * cos_t - lp[:, 1] * sin_t
                wy = ry + lp[:, 0] * sin_t + lp[:, 1] * cos_t
                lpx = ((wx - ox) / res * scale).astype(np.int32)
                lpy = (disp_h - (wy - oy) / res * scale).astype(np.int32)
                vmask = (lpx >= 0) & (lpx < disp_w) & (lpy >= 0) & (lpy < disp_h)
                vx_pts = lpx[vmask]
                vy_pts = lpy[vmask]
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        if dx * dx + dy * dy <= 4:
                            yy = np.clip(vy_pts + dy, 0, disp_h - 1)
                            xx = np.clip(vx_pts + dx, 0, disp_w - 1)
                            disp_map[yy, xx] = COLOR_LASER

            # Draw robot
            rpx = int((rx - ox) / res * scale)
            rpy = int(disp_h - (ry - oy) / res * scale)
            if 0 <= rpx < disp_w and 0 <= rpy < disp_h:
                cv2.circle(disp_map, (rpx, rpy), 12, COLOR_ROBOT, -1)
                cv2.circle(disp_map, (rpx, rpy), 12, (255, 255, 255), 2)
                ax = int(rpx + 20 * math.cos(-rt))
                ay = int(rpy + 20 * math.sin(-rt))
                cv2.arrowedLine(disp_map, (rpx, rpy), (ax, ay), (255, 255, 255), 2, tipLength=0.4)

            # Convert to GdkPixbuf
            disp_map_rgb = cv2.cvtColor(disp_map, cv2.COLOR_BGR2RGB)
            pixbuf = GdkPixbuf.Pixbuf.new_from_data(
                disp_map_rgb.tobytes(), GdkPixbuf.Colorspace.RGB, False, 8,
                disp_w, disp_h, disp_w * 3
            )
            Gdk.cairo_set_source_pixbuf(cr, pixbuf, disp_x, disp_y)
            cr.paint()

            cr.set_source_rgb(0.7, 0.7, 0.7)
            cr.set_font_size(11)
            cr.move_to(x + 10, y + h - 35)
            cr.show_text(f"Map: {mw}x{mh} @ {res:.2f}m/px | Updates: {mc}")
        else:
            cr.set_source_rgb(0.4, 0.4, 0.4)
            cr.set_font_size(18)
            cr.move_to(x + w // 2 - 100, y + h // 2)
            cr.show_text("Waiting for map...")

        cr.set_source_rgb(0.8, 0.8, 0.8)
        cr.set_font_size(12)
        cr.move_to(x + 10, y + h - 15)
        cr.show_text(f"Robot: ({rx:.2f}, {ry:.2f}) {math.degrees(rt):.0f}deg | Scans: {sc}")

    def draw_camera_panel(self, cr, x, y, w, h):
        """Draw camera panel with detection."""
        cr.set_source_rgb(0.05, 0.05, 0.05)
        cr.rectangle(x, y, w, h)
        cr.fill()

        cr.set_source_rgb(0.8, 0.4, 0)
        cr.set_line_width(2)
        cr.rectangle(x, y, w, h)
        cr.stroke()

        cr.set_source_rgb(1, 0.6, 0)
        cr.set_font_size(16)
        cr.move_to(x + 10, y + 22)
        cr.show_text("Camera + YOLO Detection")

        frame, info = state.get_camera()

        content_y = y + 35
        content_h = h - 60

        if frame is not None:
            fh, fw = frame.shape[:2]

            scale = min((w - 20) / fw, content_h / fh)
            disp_w = int(fw * scale)
            disp_h = int(fh * scale)

            disp_x = x + (w - disp_w) // 2
            disp_y = content_y + (content_h - disp_h) // 2

            disp_frame = cv2.resize(frame, (disp_w, disp_h))

            pixbuf = GdkPixbuf.Pixbuf.new_from_data(
                disp_frame.tobytes(), GdkPixbuf.Colorspace.RGB, False, 8,
                disp_w, disp_h, disp_w * 3
            )
            Gdk.cairo_set_source_pixbuf(cr, pixbuf, disp_x, disp_y)
            cr.paint()
        else:
            cr.set_source_rgb(0.4, 0.4, 0.4)
            cr.set_font_size(18)
            cr.move_to(x + w // 2 - 80, y + h // 2)
            cr.show_text("No camera feed")

        cr.set_source_rgb(0.9, 0.9, 0.9)
        cr.set_font_size(13)
        cr.move_to(x + 10, y + h - 15)
        cr.show_text(f"Detection: {info}")

    def on_quit(self, widget):
        print("[GUI] Quit requested", flush=True)
        state.request_shutdown()
        # Remove timer
        if hasattr(self, '_timer_id'):
            GLib.source_remove(self._timer_id)
        Gtk.main_quit()


# ============================================================================
# Main Entry Point with Cleanup
# ============================================================================

# Thread references for cleanup
_threads = []


def cleanup():
    """Cleanup function called on exit."""
    print("\n[Main] Cleanup started...", flush=True)
    state.request_shutdown()

    # Wait for threads with timeout
    for t in _threads:
        if t.is_alive():
            print(f"[Main] Waiting for {t.name}...", flush=True)
            t.join(timeout=2.0)
            if t.is_alive():
                print(f"[Main] Warning: {t.name} did not stop cleanly", flush=True)

    print("[Main] Cleanup complete", flush=True)


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print(f"\n[Main] Signal {sig} received", flush=True)
    state.request_shutdown()
    try:
        Gtk.main_quit()
    except:
        pass


def main():
    global _threads

    # Register cleanup handlers
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 50, flush=True)
    print("SLAM + Camera GUI - Thread Safe Version", flush=True)
    print("=" * 50, flush=True)
    print("Thread safety measures:", flush=True)
    print("  - RLock (reentrant) prevents self-deadlock")
    print("  - Lock timeout (100ms) prevents deadlock")
    print("  - Event for shutdown (atomic)")
    print("  - Proper resource cleanup", flush=True)
    print("=" * 50, flush=True)

    # Start ROS2 thread
    ros_t = threading.Thread(target=ros_thread, name="ROS2Thread", daemon=True)
    ros_t.start()
    _threads.append(ros_t)

    # Start camera thread
    cam_t = threading.Thread(target=camera_thread, name="CameraThread", daemon=True)
    cam_t.start()
    _threads.append(cam_t)

    # Wait for threads to initialize
    time.sleep(1.5)

    if not state.running:
        print("[Main] Shutdown requested during init", flush=True)
        return

    # Create and run GUI in main thread
    try:
        win = SlamCameraGUI()
        win.show_all()
        print("[Main] GUI running - press Q or ESC to quit", flush=True)
        Gtk.main()
    except Exception as e:
        print(f"[Main] GUI error: {e}", flush=True)
        state.request_shutdown()

    # Cleanup will be called by atexit


if __name__ == '__main__':
    main()
