#!/usr/bin/env python3
"""
Main Robot Control GUI - SLAM + Camera + Navigation
====================================================

Unified fullscreen GUI for RZ/V2N robot control:
- Left panel: SLAM map with robot position and LiDAR
- Right panel: Camera feed with YOLO object detection
- Bottom: Control buttons (GO, STOP) and status display

Press the "GO" button to navigate to detected bowling pins with obstacle avoidance.

Thread Architecture:
- Main thread: GTK event loop + rendering
- Thread 1: ROS2 node (map, scan, TF, navigation)
- Thread 2: Camera capture + YOLO detection

Usage:
    ros2 run bowling_target_nav main_gui
"""

import os
import subprocess
import sys

# ==========================================================================
# Display setup - MUST happen before GTK import
# ==========================================================================
def _setup_display():
    """Auto-detect and configure display for V2N (Weston/Wayland/framebuffer)."""
    weston_running = False
    try:
        result = subprocess.run(['pgrep', '-x', 'weston'], capture_output=True, timeout=2)
        weston_running = (result.returncode == 0)
    except Exception:
        pass

    if weston_running:
        os.environ['GDK_BACKEND'] = 'wayland'
        os.environ.setdefault('WAYLAND_DISPLAY', 'wayland-0')

        # Find the actual Wayland socket - Weston may run as different user
        wayland_display = os.environ.get('WAYLAND_DISPLAY', 'wayland-0')
        socket_found = False

        # Check common locations for the Wayland socket
        search_dirs = [
            os.environ.get('XDG_RUNTIME_DIR', ''),
            '/run',
            f'/run/user/{os.getuid()}',
            '/run/user/996',  # Weston user on V2N
            '/tmp',
        ]
        for d in search_dirs:
            if d and os.path.exists(os.path.join(d, wayland_display)):
                os.environ['XDG_RUNTIME_DIR'] = d
                socket_found = True
                print(f"[Display] Weston socket found: {d}/{wayland_display}", flush=True)
                break

        if not socket_found:
            os.environ.setdefault('XDG_RUNTIME_DIR', '/run')
            print(f"[Display] WARNING: Wayland socket not found, trying XDG_RUNTIME_DIR=/run", flush=True)

        print(f"[Display] Weston detected -> Wayland (XDG_RUNTIME_DIR={os.environ.get('XDG_RUNTIME_DIR')})", flush=True)
    elif os.environ.get('DISPLAY'):
        os.environ.setdefault('GDK_BACKEND', 'x11')
        print(f"[Display] X11 display: {os.environ['DISPLAY']}", flush=True)
    else:
        # Fallback: try Wayland defaults
        os.environ.setdefault('GDK_BACKEND', 'wayland')
        os.environ.setdefault('WAYLAND_DISPLAY', 'wayland-0')
        os.environ.setdefault('XDG_RUNTIME_DIR', '/run')
        print("[Display] No display server found -> trying Wayland defaults", flush=True)

    # Disable OpenCL for RZ/V2N DRP-AI compatibility
    os.environ['OPENCV_OPENCL_DEVICE'] = 'disabled'

_setup_display()

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib, GdkPixbuf

import math
import threading
import time
import cv2
import numpy as np
import signal

import atexit
from concurrent.futures import ThreadPoolExecutor

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException

# Nav2
try:
    from nav2_msgs.action import NavigateToPose
    HAS_NAV2 = True
except ImportError:
    HAS_NAV2 = False
    print("[WARNING] Nav2 not available - using direct control mode")

# DRP-AI Detector (hardware-accelerated on V2N)
try:
    from bowling_target_nav.detectors.drp_binary_detector import (
        DrpBinaryDetector, find_drp_binary, find_drp_model
    )
    HAS_DRPAI = True
except ImportError:
    HAS_DRPAI = False

# YOLO Detector (ONNX Runtime CPU fallback)
try:
    from bowling_target_nav.detectors import YoloOnnxDetector
    from bowling_target_nav.utils import DistanceEstimator
    HAS_YOLO = True
except ImportError:
    HAS_YOLO = False
    print("[WARNING] YOLO detector not available")


# ============================================================================
# Thread-safe shared state
# ============================================================================

class ThreadSafeState:
    """Thread-safe container for shared state with deadlock prevention."""

    LOCK_TIMEOUT = 0.1

    def __init__(self):
        self._lock = threading.RLock()
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
        self._laser_points = []
        self._scan_count = 0
        self._raw_scan = None  # Raw LaserScan for distance fusion
        self._raw_scan_time = 0.0

        # Camera data
        self._camera_frame = None
        self._detections = []
        self._detection_info = "Initializing..."
        self._detection_time = 0.0  # When detections were last updated

        # Navigation state
        self._nav_state = "IDLE"  # IDLE, NAVIGATING, SEARCHING, ARRIVED, ERROR
        self._nav_target = None  # (x, y, distance) of current target
        self._go_requested = False
        self._stop_requested = False
        self._last_target_time = 0.0  # When we last saw a target
        self._search_start_time = 0.0  # When we started searching

        # Obstacle info
        self._obstacle_ahead = False
        self._obstacle_dist = float('inf')

        # Tunable camera/detection params (accessed by camera thread)
        self._detect_interval = 2.0       # seconds between DRP-AI submissions
        self._detect_expiry = 1.5         # seconds before clearing cached detections
        self._confidence_threshold = 0.35  # min detection confidence

        # Distance estimator reference (for calibration)
        self._ref_box_height = 100.0
        self._ref_distance = 1.0

        # ROS node reference (set by ros_thread, used by SettingsWindow)
        self._ros_node = None

        # Errors
        self._errors = []

    def _try_lock(self, timeout=None):
        timeout = timeout or self.LOCK_TIMEOUT
        return self._lock.acquire(timeout=timeout)

    def _release_lock(self):
        try:
            self._lock.release()
        except RuntimeError:
            pass

    @property
    def running(self):
        return not self._shutdown_event.is_set()

    def request_shutdown(self):
        self._shutdown_event.set()

    # Map methods
    def set_map(self, img, info):
        if not self._try_lock():
            return False
        try:
            self._map_img = img
            self._map_info = info
            self._map_count += 1
            return True
        finally:
            self._release_lock()

    def get_map(self):
        if not self._try_lock():
            return None, None, 0
        try:
            if self._map_img is not None:
                return self._map_img.copy(), self._map_info, self._map_count
            return None, None, 0
        finally:
            self._release_lock()

    # Robot pose methods
    def set_robot_pose(self, x, y, theta):
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
        if not self._try_lock():
            return 0.0, 0.0, 0.0
        try:
            return self._robot_x, self._robot_y, self._robot_theta
        finally:
            self._release_lock()

    # Laser methods
    def set_laser(self, points):
        if not self._try_lock():
            return False
        try:
            self._laser_points = points
            self._scan_count += 1
            return True
        finally:
            self._release_lock()

    def get_laser(self):
        if not self._try_lock():
            return [], 0
        try:
            return self._laser_points[:], self._scan_count
        finally:
            self._release_lock()

    def set_raw_scan(self, scan_msg):
        if not self._try_lock():
            return False
        try:
            self._raw_scan = scan_msg
            self._raw_scan_time = time.time()
            return True
        finally:
            self._release_lock()

    def get_lidar_distance_at_angle(self, angle_rad, window=0.15):
        """Look up LiDAR range at a specific angle for distance fusion.

        Args:
            angle_rad: Target angle (0=forward, +=left, -=right)
            window: Half angular window in radians

        Returns:
            Distance in meters, or inf if no valid return
        """
        if not self._try_lock():
            return float('inf')
        try:
            scan = self._raw_scan
            if scan is None or time.time() - self._raw_scan_time > 0.5:
                return float('inf')

            angle_min_q = angle_rad - window
            angle_max_q = angle_rad + window

            start_idx = int((angle_min_q - scan.angle_min) / scan.angle_increment)
            end_idx = int((angle_max_q - scan.angle_min) / scan.angle_increment)

            num_ranges = len(scan.ranges)
            start_idx = max(0, min(num_ranges - 1, start_idx))
            end_idx = max(0, min(num_ranges - 1, end_idx))

            if start_idx > end_idx:
                return float('inf')

            valid = [
                r for r in scan.ranges[start_idx:end_idx + 1]
                if scan.range_min < r < scan.range_max
            ]
            return min(valid) if valid else float('inf')
        finally:
            self._release_lock()

    # Camera methods
    def set_camera(self, frame, detections, info, fresh_detection=False):
        if not self._try_lock():
            return False
        try:
            self._camera_frame = frame
            self._detections = detections
            self._detection_info = info
            if fresh_detection:
                self._detection_time = time.time()
            return True
        finally:
            self._release_lock()

    def get_camera(self):
        if not self._try_lock():
            return None, [], "Lock timeout", 0.0
        try:
            frame = self._camera_frame.copy() if self._camera_frame is not None else None
            det_age = time.time() - self._detection_time if self._detection_time > 0 else 999.0
            return frame, self._detections[:], self._detection_info, det_age
        finally:
            self._release_lock()

    # Navigation methods
    def set_nav_state(self, nav_state, target=None):
        if not self._try_lock():
            return False
        try:
            self._nav_state = nav_state
            if target is not None:
                self._nav_target = target
            return True
        finally:
            self._release_lock()

    def get_nav_state(self):
        if not self._try_lock():
            return "UNKNOWN", None
        try:
            return self._nav_state, self._nav_target
        finally:
            self._release_lock()

    def update_target_seen(self):
        """Call when a target is detected."""
        if not self._try_lock():
            return
        try:
            self._last_target_time = time.time()
        finally:
            self._release_lock()

    def get_time_since_target(self):
        if not self._try_lock():
            return 999.0
        try:
            if self._last_target_time == 0:
                return 999.0
            return time.time() - self._last_target_time
        finally:
            self._release_lock()

    def start_search(self):
        if not self._try_lock():
            return
        try:
            self._search_start_time = time.time()
            self._nav_state = "SEARCHING"
        finally:
            self._release_lock()

    def get_search_time(self):
        if not self._try_lock():
            return 0.0
        try:
            if self._search_start_time == 0:
                return 0.0
            return time.time() - self._search_start_time
        finally:
            self._release_lock()

    def request_go(self):
        if not self._try_lock():
            return
        try:
            self._go_requested = True
        finally:
            self._release_lock()

    def request_stop(self):
        if not self._try_lock():
            return
        try:
            self._stop_requested = True
        finally:
            self._release_lock()

    def check_and_clear_go(self):
        if not self._try_lock():
            return False
        try:
            val = self._go_requested
            self._go_requested = False
            return val
        finally:
            self._release_lock()

    def check_and_clear_stop(self):
        if not self._try_lock():
            return False
        try:
            val = self._stop_requested
            self._stop_requested = False
            return val
        finally:
            self._release_lock()

    def set_obstacle(self, ahead, dist):
        if not self._try_lock():
            return
        try:
            self._obstacle_ahead = ahead
            self._obstacle_dist = dist
        finally:
            self._release_lock()

    def get_obstacle(self):
        if not self._try_lock():
            return False, float('inf')
        try:
            return self._obstacle_ahead, self._obstacle_dist
        finally:
            self._release_lock()

    # Tunable parameter accessors (thread-safe)
    def set_detect_interval(self, val):
        if self._try_lock():
            try: self._detect_interval = val
            finally: self._release_lock()

    def get_detect_interval(self):
        if self._try_lock():
            try: return self._detect_interval
            finally: self._release_lock()
        return 2.0

    def set_detect_expiry(self, val):
        if self._try_lock():
            try: self._detect_expiry = val
            finally: self._release_lock()

    def get_detect_expiry(self):
        if self._try_lock():
            try: return self._detect_expiry
            finally: self._release_lock()
        return 1.5

    def set_confidence_threshold(self, val):
        if self._try_lock():
            try: self._confidence_threshold = val
            finally: self._release_lock()

    def get_confidence_threshold(self):
        if self._try_lock():
            try: return self._confidence_threshold
            finally: self._release_lock()
        return 0.35

    def set_calibration(self, box_height, distance):
        if self._try_lock():
            try:
                self._ref_box_height = box_height
                self._ref_distance = distance
            finally: self._release_lock()

    def get_calibration(self):
        if self._try_lock():
            try: return self._ref_box_height, self._ref_distance
            finally: self._release_lock()
        return 100.0, 1.0

    def add_error(self, source, error):
        if not self._try_lock():
            return
        try:
            self._errors.append(f"{source}: {error}")
            if len(self._errors) > 10:
                self._errors = self._errors[-10:]
        finally:
            self._release_lock()


# Global state
state = ThreadSafeState()

# Colors (BGR for OpenCV)
COLOR_FREE = (50, 50, 50)
COLOR_OCCUPIED = (0, 200, 255)
COLOR_UNKNOWN = (30, 30, 30)
COLOR_ROBOT = (0, 255, 0)
COLOR_LASER = (0, 0, 255)
COLOR_TARGET = (255, 0, 255)
COLOR_GRID = (60, 60, 60)


def find_model_path():
    """Find YOLO model in standard locations (prefers YOLOv8m)."""
    # Search for new model first, then fall back to old
    model_names = ['bowling_yolov8m_int8.onnx', 'bowling_yolov8m.onnx', 'bowling_yolov5.onnx']
    search_dirs = [
        os.path.join(os.path.dirname(__file__), '..', '..', 'models'),
        os.path.expanduser('~/ros2_ws/install/bowling_target_nav/share/bowling_target_nav/models'),
        os.path.expanduser('~/ros2_ws/src/bowling_target_nav/models'),
        os.path.expanduser('~/ros2_ws/build/bowling_target_nav/models'),
        '/opt/ros/humble/share/bowling_target_nav/models',
        '/tmp',
    ]
    for name in model_names:
        for d in search_dirs:
            p = os.path.abspath(os.path.join(d, name))
            if os.path.exists(p):
                return p
    return None

# Target class names matching the ONNX model metadata
TARGET_CLASS = 'bowling-pins'
FILTER_CLASSES = ['bowling-pins']


# ============================================================================
# ROS2 Node Thread
# ============================================================================

class MainGuiNode(Node):
    """ROS2 node for map, scan, TF, and navigation control."""

    def __init__(self):
        super().__init__('main_gui_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS profiles
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, sensor_qos)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Remote command subscriber (allows triggering GO/STOP via topic)
        self.create_subscription(String, '/gui/command', self.command_cb, 10)

        # Nav2 action client
        self.nav_client = None
        self.current_goal_handle = None
        if HAS_NAV2:
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Parameters
        self.approach_distance = 0.15  # Stop 15cm from target
        self.linear_speed = 0.15
        self.min_linear_speed = 0.10   # Dead zone: minimum speed motors can actually move
        self.angular_speed = 0.5
        self.obstacle_distance = 0.25  # Stop if obstacle closer
        self.obstacle_slowdown_distance = 0.5  # Slow down zone
        self.robot_half_width = 0.15  # Half robot width for corridor check
        self.search_angular_speed = 0.4  # Rotation speed when searching
        self.lost_timeout = 3.0  # Seconds without target before searching
        self.search_timeout = 30.0  # Max seconds to search before giving up

        # Current target from detections
        self.current_target = None  # (x, y, theta) in base_link frame
        self.is_active = False  # True when GO is pressed and robot is active
        self.last_target_angle = 0.0  # Last known target angle for search direction
        self.last_target = None  # Last detected target dict (for continued navigation)

        # Blind approach: dead-reckon to last known pin position when camera loses it
        self.blind_approach_active = False
        self.blind_approach_start_time = 0.0
        self.blind_approach_start_pose = (0.0, 0.0, 0.0)
        self.blind_approach_target_map = (0.0, 0.0)
        self.blind_approach_target_distance = 0.0
        self.blind_approach_entry_distance = 0.80  # enter blind mode if last dist < this
        self.blind_approach_speed = 0.10            # m/s (slower for safety)
        self.blind_approach_timeout = 8.0           # max seconds in blind mode
        self.blind_approach_lidar_stop = 0.12       # LiDAR emergency stop distance
        self.blind_approach_arrival_margin = 0.10   # stop this far from estimated target

        # Control timer - 20Hz for responsive navigation
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Main GUI ROS node started')

    def map_cb(self, msg):
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
        if not state.running:
            return
        try:
            points = []
            angle = msg.angle_min
            for r in msg.ranges:
                if msg.range_min < r < msg.range_max:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    points.append((x, y))
                angle += msg.angle_increment
            state.set_laser(points)
            state.set_raw_scan(msg)

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
                pass
        except Exception as e:
            state.add_error('ros_scan', str(e))

    def command_cb(self, msg):
        """Handle remote commands via /gui/command topic."""
        cmd = msg.data.strip().lower()
        if cmd == 'go':
            self.get_logger().info("Remote GO command received")
            state.request_go()
        elif cmd == 'stop':
            self.get_logger().info("Remote STOP command received")
            state.request_stop()

    def control_loop(self):
        """Main control loop - handles GO/STOP requests and search behavior."""
        # Check for stop request
        if state.check_and_clear_stop():
            self._stop_robot()
            self.is_active = False
            self.blind_approach_active = False
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            state.set_nav_state("IDLE")
            self.get_logger().info("Navigation stopped")
            return

        # Check for go request - activates continuous tracking
        if state.check_and_clear_go():
            self.is_active = True
            self.blind_approach_active = False
            state.set_nav_state("IDLE", None)
            self.current_goal_handle = None
            self.get_logger().info("Navigation activated - looking for targets")

        # If not active, do nothing
        if not self.is_active:
            return

        # Get current detections with age
        _, detections, _, det_age = state.get_camera()

        # Only use detections that are fresh (< 0.5s old)
        if det_age > 0.5:
            detections = []

        target = self._find_best_target(detections)

        nav_state, _ = state.get_nav_state()

        if target:
            # Target found with FRESH detection! Update last seen time
            state.update_target_seen()
            self.last_target = target

            # Cancel blind approach on re-detection
            if self.blind_approach_active:
                self.get_logger().info("Target re-detected during BLIND_APPROACH")
                self.blind_approach_active = False

            if nav_state in ("SEARCHING", "BLIND_APPROACH"):
                self.get_logger().info("Target found!")

            # Don't re-navigate if already arrived
            if nav_state == "ARRIVED":
                return

            # Navigate to target - continuously update for direct mode
            self._navigate_to_target(target)

        else:
            # No target visible
            time_since_target = state.get_time_since_target()

            # Blind approach active: continue dead-reckoning toward pin
            if self.blind_approach_active:
                self._blind_approach_step()
                return

            if nav_state == "NAVIGATING" and time_since_target <= self.lost_timeout and self.last_target:
                # Slow down while waiting for fresh detection (don't use stale coordinates)
                cmd = Twist()
                cmd.linear.x = self.min_linear_speed  # Creep forward at minimum motor speed
                self.cmd_vel_pub.publish(cmd)
                return

            if nav_state == "NAVIGATING" and time_since_target > self.lost_timeout:
                # Lost timeout expired — decide: blind approach or search
                _, nav_target = state.get_nav_state()
                last_distance = nav_target[2] if nav_target else float('inf')

                if last_distance < self.blind_approach_entry_distance:
                    # Close enough: dead-reckon to the pin instead of searching
                    if self._enter_blind_approach():
                        self._blind_approach_step()
                        return

                # Far away or blind approach entry failed: search as before
                self.get_logger().warn(f"Target lost for {time_since_target:.1f}s - searching...")
                if self.current_goal_handle:
                    self.current_goal_handle.cancel_goal_async()
                    self.current_goal_handle = None
                state.start_search()
                self._search_rotate()

            elif nav_state == "SEARCHING":
                search_time = state.get_search_time()
                if search_time > self.search_timeout:
                    self.get_logger().warn(f"Search timeout ({search_time:.1f}s) - giving up")
                    self._stop_robot()
                    self.is_active = False
                    state.set_nav_state("IDLE", None)
                else:
                    self._search_rotate()

            elif nav_state == "IDLE" and self.is_active:
                self.get_logger().info("No target visible - searching...")
                state.start_search()
                self._search_rotate()

    def _search_rotate(self):
        """Rotate in place to search for targets (towards last known direction)."""
        cmd = Twist()
        direction = 1.0 if self.last_target_angle >= 0 else -1.0

        # Check obstacles to avoid rotating into walls
        min_front, left_free, right_free = self._check_obstacles()
        if min_front < self.obstacle_distance:
            # Don't rotate towards blocked side
            if direction > 0 and not left_free:
                direction = -1.0
            elif direction < 0 and not right_free:
                direction = 1.0

        cmd.angular.z = self.search_angular_speed * direction
        self.cmd_vel_pub.publish(cmd)

    def _find_best_target(self, detections):
        """Find the best target from detections (closest bowling pin)."""
        best = None
        best_dist = float('inf')

        for det in detections:
            # All detections are already filtered to pin classes by detector
            dist = det.get('distance', float('inf'))
            if dist < best_dist:
                best_dist = dist
                best = det
        return best

    def _navigate_to_target(self, target):
        """Start navigation to target with LiDAR+Vision distance fusion."""
        vision_distance = target.get('distance', 1.0)
        angle = target.get('angle', 0.0)
        # Convert camera convention (+=right) to ROS convention (+=left)
        angle = -angle
        self.last_target_angle = angle  # Remember for search direction

        # LiDAR+Vision fusion: prefer LiDAR distance at vision-detected angle
        lidar_distance = state.get_lidar_distance_at_angle(angle)
        if lidar_distance < float('inf'):
            distance = lidar_distance
            source = "lidar"
        else:
            distance = vision_distance
            source = "vision"

        # Convert to position in base_link frame
        target_x = distance * math.cos(angle)
        target_y = distance * math.sin(angle)

        prev_state, _ = state.get_nav_state()
        state.set_nav_state("NAVIGATING", (target_x, target_y, distance))
        # Only log on state change or every 2 seconds
        if prev_state != "NAVIGATING" or not hasattr(self, '_last_nav_log') or time.time() - self._last_nav_log > 2.0:
            self.get_logger().info(f"Target at {distance:.2f}m ({source}), {math.degrees(angle):.1f}deg")
            self._last_nav_log = time.time()

        if HAS_NAV2 and self.nav_client and self.nav_client.server_is_ready():
            self._send_nav2_goal(target_x, target_y)
        else:
            # Direct control mode
            self._direct_navigate(target_x, target_y, distance)

    def _send_nav2_goal(self, target_x, target_y):
        """Send navigation goal to Nav2."""
        try:
            # Get current robot position in map frame
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            q = transform.transform.rotation
            robot_theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            # Transform target to map frame
            cos_t = math.cos(robot_theta)
            sin_t = math.sin(robot_theta)
            map_target_x = robot_x + target_x * cos_t - target_y * sin_t
            map_target_y = robot_y + target_x * sin_t + target_y * cos_t

            # Compute approach point (stop before target)
            dx = map_target_x - robot_x
            dy = map_target_y - robot_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist > self.approach_distance:
                ux, uy = dx / dist, dy / dist
                goal_x = map_target_x - self.approach_distance * ux
                goal_y = map_target_y - self.approach_distance * uy
            else:
                goal_x, goal_y = robot_x, robot_y
                state.set_nav_state("ARRIVED")
                return

            # Face the target
            yaw = math.atan2(map_target_y - goal_y, map_target_x - goal_x)

            # Create Nav2 goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = goal_x
            goal_msg.pose.pose.position.y = goal_y
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

            future = self.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self._nav2_goal_response)

        except Exception as e:
            self.get_logger().error(f"Nav2 goal error: {e}")
            state.set_nav_state("ERROR")

    def _nav2_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            state.set_nav_state("ERROR")
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_goal_result)

    def _nav2_goal_result(self, future):
        self.current_goal_handle = None
        state.set_nav_state("ARRIVED")
        self.get_logger().info("Navigation completed")

    def _direct_navigate(self, target_x, target_y, distance):
        """Direct velocity control with LiDAR obstacle avoidance."""
        angle = math.atan2(target_y, target_x)
        cmd = Twist()

        if distance <= self.approach_distance:
            self.get_logger().info(f"ARRIVED: dist={distance:.2f}m")
            state.set_nav_state("ARRIVED")
            state.set_obstacle(False, float('inf'))
            self.cmd_vel_pub.publish(cmd)
            return

        # Check for obstacles using LiDAR
        min_front, left_free, right_free = self._check_obstacles()
        state.set_obstacle(min_front < self.obstacle_distance, min_front)

        if min_front < self.obstacle_distance:
            # Obstacle in front!
            if min_front < 0.15:
                # Emergency stop - too close
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            elif left_free and (not right_free or angle > 0):
                # Steer left (towards target if possible)
                cmd.angular.z = self.angular_speed * 0.8
                cmd.linear.x = 0.0  # Don't move forward while turning away
            elif right_free:
                # Steer right
                cmd.angular.z = -self.angular_speed * 0.8
                cmd.linear.x = 0.0
            else:
                # Both sides blocked - back up slightly
                cmd.linear.x = -self.linear_speed * 0.2
                cmd.angular.z = 0.0

            if not hasattr(self, '_last_obs_log') or time.time() - self._last_obs_log > 1.0:
                self.get_logger().warn(
                    f"OBSTACLE at {min_front:.2f}m! L:{left_free} R:{right_free}")
                self._last_obs_log = time.time()
        else:
            # Normal proportional navigation
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, angle * 1.0))
            turn_factor = 1.0 - min(1.0, abs(angle) / 0.5)
            cmd.linear.x = self.linear_speed * max(0.3, turn_factor)

            # Slow down when approaching obstacles
            if min_front < self.obstacle_slowdown_distance:
                slow_factor = (min_front - self.obstacle_distance) / (
                    self.obstacle_slowdown_distance - self.obstacle_distance)
                cmd.linear.x *= max(0.3, slow_factor)

            # Enforce minimum speed (motor dead zone)
            if cmd.linear.x > 0:
                cmd.linear.x = max(cmd.linear.x, self.min_linear_speed)

        self.cmd_vel_pub.publish(cmd)

    def _enter_blind_approach(self):
        """Enter blind approach mode: dead-reckon to last known pin position.

        Converts the last known target from base_link frame to map frame
        so we can track progress via odometry as we drive toward it.
        Returns True if successfully entered, False otherwise.
        """
        robot_x, robot_y, robot_theta = state.get_robot_pose()
        _, nav_target = state.get_nav_state()
        if nav_target is None:
            return False

        target_x_bl, target_y_bl, distance = nav_target

        # Transform base_link target to map frame
        cos_t = math.cos(robot_theta)
        sin_t = math.sin(robot_theta)
        target_x_map = robot_x + target_x_bl * cos_t - target_y_bl * sin_t
        target_y_map = robot_y + target_x_bl * sin_t + target_y_bl * cos_t

        self.blind_approach_active = True
        self.blind_approach_start_time = time.time()
        self.blind_approach_start_pose = (robot_x, robot_y, robot_theta)
        self.blind_approach_target_map = (target_x_map, target_y_map)
        self.blind_approach_target_distance = distance

        state.set_nav_state("BLIND_APPROACH", nav_target)
        self.get_logger().info(
            f"BLIND_APPROACH: target_map=({target_x_map:.2f}, {target_y_map:.2f}), "
            f"distance={distance:.2f}m")
        return True

    def _blind_approach_step(self):
        """Execute one tick of blind approach dead-reckoning.

        Drives toward the map-frame target using odometry feedback.
        Checks exit conditions: arrival, timeout, LiDAR obstacle, heading divergence.
        """
        robot_x, robot_y, robot_theta = state.get_robot_pose()
        target_x, target_y = self.blind_approach_target_map

        # Compute remaining distance and heading to target
        dx = target_x - robot_x
        dy = target_y - robot_y
        remaining = math.sqrt(dx * dx + dy * dy)

        desired_heading = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(desired_heading - robot_theta),
            math.cos(desired_heading - robot_theta))

        elapsed = time.time() - self.blind_approach_start_time

        # Exit: arrived at estimated position
        if remaining <= self.blind_approach_arrival_margin:
            self.get_logger().info(
                f"BLIND_APPROACH: Arrived! remaining={remaining:.3f}m, elapsed={elapsed:.1f}s")
            self._stop_robot()
            self.blind_approach_active = False
            state.set_nav_state("ARRIVED")
            return

        # Exit: timeout
        if elapsed > self.blind_approach_timeout:
            self.get_logger().warn(
                f"BLIND_APPROACH: Timeout after {elapsed:.1f}s, remaining={remaining:.3f}m")
            self._stop_robot()
            self.blind_approach_active = False
            state.start_search()
            return

        # Exit: LiDAR detects very close obstacle (likely the pin)
        min_front, _, _ = self._check_obstacles()
        if min_front < self.blind_approach_lidar_stop:
            self.get_logger().info(
                f"BLIND_APPROACH: LiDAR stop! obstacle at {min_front:.3f}m, "
                f"remaining={remaining:.3f}m")
            self._stop_robot()
            self.blind_approach_active = False
            state.set_nav_state("ARRIVED")
            return

        # Exit: heading diverged too much (odom failure or collision)
        if abs(heading_error) > math.radians(45):
            self.get_logger().warn(
                f"BLIND_APPROACH: Heading diverged {math.degrees(heading_error):.1f}deg, aborting")
            self._stop_robot()
            self.blind_approach_active = False
            state.start_search()
            return

        # Drive toward target with proportional steering
        cmd = Twist()
        speed_ramp = min(1.0, remaining / 0.30)  # ramp down in last 30cm
        cmd.linear.x = max(self.blind_approach_speed * max(0.5, speed_ramp), self.min_linear_speed)
        cmd.angular.z = max(-0.2, min(0.2, 0.8 * heading_error))
        self.cmd_vel_pub.publish(cmd)

    def _stop_robot(self):
        """Stop all robot motion."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def _check_obstacles(self):
        """Check LiDAR for obstacles in front/left/right of robot.

        Returns (min_front_dist, left_free, right_free).
        Laser points are in laser frame (~= base_link): +x=forward, +y=left.
        """
        points, _ = state.get_laser()

        obs_dist = self.obstacle_distance          # 0.25m danger zone
        slow_dist = self.obstacle_slowdown_distance  # 0.5m slow zone
        hw = self.robot_half_width                   # 0.15m

        min_front = float('inf')
        left_free = True
        right_free = True

        for x, y in points:
            if x <= 0 or x > slow_dist:
                continue

            # Front corridor: within robot width
            if abs(y) < hw:
                min_front = min(min_front, x)

            # Left-front zone: check if safe to steer left (wider zone)
            if x < slow_dist and hw <= y < hw + 0.4:
                left_free = False

            # Right-front zone: check if safe to steer right (wider zone)
            if x < slow_dist and -(hw + 0.4) < y <= -hw:
                right_free = False

        return min_front, left_free, right_free


def ros_thread():
    """ROS2 spin thread."""
    node = None
    print("[ROS Thread] Starting...", flush=True)

    try:
        rclpy.init()
        node = MainGuiNode()
        state._ros_node = node

        while state.running:
            try:
                rclpy.spin_once(node, timeout_sec=0.02)
            except Exception as e:
                if state.running:
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
# Camera + Detection Thread (async detection for real-time performance)
# ============================================================================

def _detect_objects(frame, detector, estimator):
    """Run YOLO detection on a frame (called in background thread).

    Returns (detections_list, inference_time_seconds).
    """
    detections = []
    infer_time = 0.0
    try:
        result = detector.detect(frame)
        infer_time = result.inference_time if result.inference_time else 0.0

        if result.success:
            for det in result.detections:
                det_dict = {
                    'class_name': det.class_name,
                    'confidence': det.confidence,
                    'bbox': det.bbox,
                }
                if estimator:
                    distance, angle = estimator.estimate(det)
                    det_dict['distance'] = distance
                    det_dict['angle'] = angle
                detections.append(det_dict)
    except Exception as e:
        print(f"[Detection] Error: {e}", flush=True)
    return detections, infer_time


def _draw_rect_np(img, x1, y1, x2, y2, color, thickness=2):
    """Draw a rectangle using numpy (avoids OpenCV DRP acceleration on V2N)."""
    h, w = img.shape[:2]
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = min(w - 1, int(x2)), min(h - 1, int(y2))
    b, g, r = int(color[0]), int(color[1]), int(color[2])
    t = thickness
    # Top edge
    img[y1:y1+t, x1:x2, :] = (b, g, r)
    # Bottom edge
    img[y2-t:y2, x1:x2, :] = (b, g, r)
    # Left edge
    img[y1:y2, x1:x1+t, :] = (b, g, r)
    # Right edge
    img[y1:y2, x2-t:x2, :] = (b, g, r)


def camera_thread():
    """Camera capture with async YOLO detection (non-blocking).

    Camera runs at full speed for smooth display.
    YOLO detection runs in a background thread - results appear as soon as ready.
    This decouples display FPS from detection speed.
    """
    cap = None
    detector = None
    executor = None
    print("[Camera Thread] Starting...", flush=True)

    # Disable OpenCL for RZ/V2N
    cv2.ocl.setUseOpenCL(False)

    try:
        estimator = None
        use_drpai = False

        # Try DRP-AI hardware accelerator first (V2N only)
        if HAS_DRPAI:
            drp_binary = find_drp_binary()
            drp_model = find_drp_model()
            if drp_binary and drp_model:
                # Disable OpenCV OCA (DRP acceleration) BEFORE starting DRP-AI
                # to prevent hardware contention. Done here in camera thread
                # (not at import time) so GTK/Weston are already initialized.
                try:
                    import ctypes
                    lib = ctypes.CDLL('/usr/lib/libopencv_imgproc.so.4.9.0')
                    func = lib['_Z12OCA_ActivatePm']
                    func.argtypes = [ctypes.POINTER(ctypes.c_ulong)]
                    func.restype = ctypes.c_int
                    oca_list = (ctypes.c_ulong * 16)(*([0] * 16))
                    func(oca_list)
                    print("[Camera Thread] OCA disabled (DRP-AI conflict prevention)", flush=True)
                except Exception:
                    pass

                print(f"[Camera Thread] Loading DRP-AI detector: {drp_binary}", flush=True)
                print(f"[Camera Thread] DRP-AI model: {drp_model}", flush=True)
                try:
                    detector = DrpBinaryDetector(
                        binary_path=drp_binary,
                        model_dir=drp_model,
                        confidence_threshold=0.35,
                        target_class=TARGET_CLASS,
                    )
                    if detector.initialize():
                        use_drpai = True
                        estimator = DistanceEstimator(
                            reference_box_height=100.0,
                            reference_distance=1.0,
                            frame_width=640,
                            frame_height=480,
                            horizontal_fov=60.0
                        )
                        print(f"[Camera Thread] DRP-AI detector ready (hardware accelerated)", flush=True)
                    else:
                        print("[Camera Thread] DRP-AI detector init failed, trying ONNX fallback", flush=True)
                        detector = None
                except Exception as e:
                    print(f"[Camera Thread] DRP-AI load failed: {e}, trying ONNX fallback", flush=True)
                    detector = None
            else:
                if not drp_binary:
                    print("[Camera Thread] DRP-AI binary not found", flush=True)
                if not drp_model:
                    print("[Camera Thread] DRP-AI model not found", flush=True)

        # Fallback: ONNX Runtime on CPU
        if detector is None:
            model_path = find_model_path()
            if model_path and HAS_YOLO:
                print(f"[Camera Thread] Loading ONNX model: {model_path}", flush=True)
                try:
                    detector = YoloOnnxDetector(
                        model_path=model_path,
                        confidence_threshold=0.35,
                        nms_threshold=0.45,
                        target_class=TARGET_CLASS,
                        filter_classes=FILTER_CLASSES,
                    )
                    if not detector.initialize():
                        print("[Camera Thread] ONNX detector init failed", flush=True)
                        detector = None
                    else:
                        estimator = DistanceEstimator(
                            reference_box_height=100.0,
                            reference_distance=1.0,
                            frame_width=640,
                            frame_height=480,
                            horizontal_fov=60.0
                        )
                        print(f"[Camera Thread] ONNX detector ready (CPU), filter={FILTER_CLASSES}", flush=True)
                except Exception as e:
                    print(f"[Camera Thread] ONNX load failed: {e}", flush=True)
                    detector = None
            else:
                if not model_path:
                    print("[Camera Thread] No ONNX model found", flush=True)
                if not HAS_YOLO:
                    print("[Camera Thread] ONNX Runtime not available", flush=True)

        # Open camera
        for attempt in range(3):
            if not state.running:
                return
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if cap.isOpened():
                print("[Camera Thread] Camera opened", flush=True)
                break
            print(f"[Camera Thread] Camera open attempt {attempt+1}/3 failed", flush=True)
            time.sleep(1)

        if not cap or not cap.isOpened():
            state.set_camera(None, [], "Camera not available")
            print("[Camera Thread] ERROR: Cannot open camera", flush=True)
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Async detection with DRP-AI / display synchronization
        executor = ThreadPoolExecutor(max_workers=1)
        pending_future = None
        cached_detections = []

        # DRP-AI rate limiter: interval is tunable via Settings window
        last_submit_time = 0.0

        # Temporal tracking: keep last known detections during brief gaps
        consecutive_misses = 0
        max_misses = 5  # Match config: consecutive_misses_to_lose

        frame_count = 0
        last_fps_time = time.time()
        last_detection_time = time.time()
        fps = 0.0

        while state.running:
            # Flush V4L2 buffer to always get the latest frame
            cap.grab()
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            frame_count += 1

            # Check if background detection finished
            fresh = False
            if pending_future is not None and pending_future.done():
                try:
                    new_detections, infer_time = pending_future.result()
                except Exception:
                    new_detections, infer_time = [], 0.0
                pending_future = None
                fresh = True
                last_detection_time = time.time()

                # Temporal tracking: tolerate brief detection gaps
                if new_detections:
                    cached_detections = new_detections
                    consecutive_misses = 0
                else:
                    consecutive_misses += 1
                    if consecutive_misses >= max_misses:
                        cached_detections = []

            # Expire old detections (expiry tunable via Settings)
            detect_expiry = state.get_detect_expiry()
            detection_age = time.time() - last_detection_time
            if detection_age > detect_expiry:
                cached_detections = []

            # Submit new detection if detector is idle and rate limit elapsed
            now = time.time()
            if pending_future is None and detector:
                detect_interval = state.get_detect_interval()
                if not use_drpai or (now - last_submit_time) >= detect_interval:
                    pending_future = executor.submit(
                        _detect_objects, frame.copy(), detector, estimator)
                    last_submit_time = now

            # Draw cached detections on current frame
            display_frame = frame.copy()
            info = "No pins detected"

            if cached_detections and detection_age < detect_expiry:
                # Draw boxes - color indicates freshness
                box_color = (0, 255, 0) if detection_age < 0.3 else (0, 200, 200)
                for det in cached_detections:
                    x1, y1, x2, y2 = det['bbox']
                    conf = det['confidence']
                    if use_drpai:
                        # Numpy drawing to avoid OpenCV DRP conflict on V2N
                        _draw_rect_np(display_frame, x1, y1, x2, y2, box_color, 2)
                    else:
                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), box_color, 2)
                        cv2.putText(display_frame, f"Pin {conf:.2f}", (x1, y1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                        if 'distance' in det:
                            cv2.putText(display_frame, f"{det['distance']:.2f}m",
                                        (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

                best = min(cached_detections, key=lambda d: d.get('distance', 999))
                info = f"Pins: {best['distance']:.2f}m, {math.degrees(best['angle']):.1f}deg"

            # FPS counter
            now = time.time()
            if now - last_fps_time >= 1.0:
                fps = frame_count / (now - last_fps_time)
                frame_count = 0
                last_fps_time = now

            if not use_drpai:
                cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Prepare display frame (numpy BGR->RGB, no OpenCV)
            rgb_frame = display_frame[:, :, ::-1].copy()

            state.set_camera(rgb_frame,
                             cached_detections if detection_age < detect_expiry else [],
                             info, fresh_detection=fresh)

            time.sleep(0.001)

    except Exception as e:
        state.add_error('camera_thread', str(e))
        print(f"[Camera Thread] ERROR: {e}", flush=True)
    finally:
        print("[Camera Thread] Cleaning up...", flush=True)
        if executor:
            executor.shutdown(wait=False)
        if cap:
            cap.release()
        if detector:
            detector.shutdown()
        print("[Camera Thread] Stopped", flush=True)


# ============================================================================
# Settings Window
# ============================================================================

class SettingsWindow(Gtk.Window):
    """Settings popup with parameter sliders and calibration tools."""

    def __init__(self):
        super().__init__(title="Robot Settings")
        self.set_default_size(540, 620)
        self.set_position(Gtk.WindowPosition.CENTER)
        self.set_type_hint(Gdk.WindowTypeHint.DIALOG)
        self.get_style_context().add_class('settings-window')

        # Don't destroy on close — just hide so sliders keep their values
        self.connect('delete-event', self._on_delete)

        self._motor_test_timer = None

        notebook = Gtk.Notebook()
        self.add(notebook)

        notebook.append_page(self._build_nav_tab(), Gtk.Label(label="Navigation"))
        notebook.append_page(self._build_blind_tab(), Gtk.Label(label="Blind Approach"))
        notebook.append_page(self._build_detection_tab(), Gtk.Label(label="Detection"))
        notebook.append_page(self._build_obstacle_tab(), Gtk.Label(label="Obstacle"))
        notebook.append_page(self._build_calibration_tab(), Gtk.Label(label="Calibration"))

    def _on_delete(self, widget, event):
        self.hide()
        return True  # Prevent destroy

    # --- Slider helper ---
    def _make_slider(self, label_text, min_val, max_val, step, default, callback):
        """Create a labeled slider row that calls callback(value) on change."""
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        row.set_margin_start(10)
        row.set_margin_end(10)
        row.set_margin_top(4)
        row.set_margin_bottom(4)

        label = Gtk.Label(label=label_text)
        label.set_xalign(0)
        label.set_size_request(200, -1)
        row.pack_start(label, False, False, 0)

        adj = Gtk.Adjustment(value=default, lower=min_val, upper=max_val,
                             step_increment=step, page_increment=step * 5)
        scale = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL, adjustment=adj)
        scale.set_digits(2 if step < 0.1 else 1)
        scale.set_value_pos(Gtk.PositionType.RIGHT)
        row.pack_start(scale, True, True, 0)

        val_label = Gtk.Label(label=f"{default:.2f}")
        val_label.set_size_request(60, -1)
        row.pack_start(val_label, False, False, 0)

        def on_change(s):
            v = s.get_value()
            val_label.set_text(f"{v:.2f}")
            callback(v)

        scale.connect('value-changed', on_change)
        return row

    def _make_section_label(self, text):
        label = Gtk.Label()
        label.set_markup(f"<span foreground='#58a6ff' font_weight='bold' size='large'>{text}</span>")
        label.set_xalign(0)
        label.set_margin_start(10)
        label.set_margin_top(14)
        label.set_margin_bottom(6)
        return label

    # --- Tab builders ---
    def _build_nav_tab(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        def _set(attr):
            def cb(v):
                node = state._ros_node
                if node:
                    setattr(node, attr, v)
            return cb

        box.pack_start(self._make_slider("Linear Speed (m/s)", 0.05, 0.30, 0.01, 0.15, _set('linear_speed')), False, False, 0)
        box.pack_start(self._make_slider("Min Speed (m/s)", 0.05, 0.20, 0.01, 0.10, _set('min_linear_speed')), False, False, 0)
        box.pack_start(self._make_slider("Angular Speed (rad/s)", 0.1, 1.0, 0.05, 0.5, _set('angular_speed')), False, False, 0)
        box.pack_start(self._make_slider("Approach Distance (m)", 0.05, 0.50, 0.01, 0.15, _set('approach_distance')), False, False, 0)
        box.pack_start(self._make_slider("Lost Timeout (s)", 1.0, 10.0, 0.5, 3.0, _set('lost_timeout')), False, False, 0)
        box.pack_start(self._make_slider("Search Timeout (s)", 10.0, 60.0, 5.0, 30.0, _set('search_timeout')), False, False, 0)
        box.pack_start(self._make_slider("Search Angular Speed", 0.1, 1.0, 0.05, 0.4, _set('search_angular_speed')), False, False, 0)
        return box

    def _build_blind_tab(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        def _set(attr):
            def cb(v):
                node = state._ros_node
                if node:
                    setattr(node, attr, v)
            return cb

        box.pack_start(self._make_slider("Entry Distance (m)", 0.30, 1.50, 0.05, 0.80, _set('blind_approach_entry_distance')), False, False, 0)
        box.pack_start(self._make_slider("Approach Speed (m/s)", 0.05, 0.20, 0.01, 0.10, _set('blind_approach_speed')), False, False, 0)
        box.pack_start(self._make_slider("Timeout (s)", 3.0, 15.0, 0.5, 8.0, _set('blind_approach_timeout')), False, False, 0)
        box.pack_start(self._make_slider("LiDAR Stop (m)", 0.05, 0.30, 0.01, 0.12, _set('blind_approach_lidar_stop')), False, False, 0)
        box.pack_start(self._make_slider("Arrival Margin (m)", 0.05, 0.25, 0.01, 0.10, _set('blind_approach_arrival_margin')), False, False, 0)
        return box

    def _build_detection_tab(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        box.pack_start(self._make_slider("Confidence Threshold", 0.10, 0.90, 0.05, 0.35,
            lambda v: state.set_confidence_threshold(v)), False, False, 0)
        box.pack_start(self._make_slider("Detection Interval (s)", 0.5, 5.0, 0.1, 2.0,
            lambda v: state.set_detect_interval(v)), False, False, 0)
        box.pack_start(self._make_slider("Detection Expiry (s)", 0.5, 5.0, 0.1, 1.5,
            lambda v: state.set_detect_expiry(v)), False, False, 0)
        return box

    def _build_obstacle_tab(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        def _set(attr):
            def cb(v):
                node = state._ros_node
                if node:
                    setattr(node, attr, v)
            return cb

        box.pack_start(self._make_slider("Stop Distance (m)", 0.10, 0.50, 0.01, 0.25, _set('obstacle_distance')), False, False, 0)
        box.pack_start(self._make_slider("Slowdown Distance (m)", 0.20, 1.00, 0.05, 0.50, _set('obstacle_slowdown_distance')), False, False, 0)
        box.pack_start(self._make_slider("Robot Half Width (m)", 0.05, 0.30, 0.01, 0.15, _set('robot_half_width')), False, False, 0)
        return box

    def _build_calibration_tab(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        # --- Distance Calibration ---
        box.pack_start(self._make_section_label("Distance Calibration"), False, False, 0)

        info = Gtk.Label(label="Place pin at known distance, then press Calibrate")
        info.set_xalign(0)
        info.set_margin_start(10)
        box.pack_start(info, False, False, 4)

        cal_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        cal_row.set_margin_start(10)
        cal_row.set_margin_end(10)

        cal_row.pack_start(Gtk.Label(label="Known Distance (m):"), False, False, 0)
        self.cal_distance_spin = Gtk.SpinButton.new_with_range(0.3, 3.0, 0.1)
        self.cal_distance_spin.set_value(1.0)
        cal_row.pack_start(self.cal_distance_spin, False, False, 0)

        cal_btn = Gtk.Button(label="CALIBRATE")
        cal_btn.connect('clicked', self._on_calibrate)
        cal_row.pack_start(cal_btn, False, False, 10)

        self.cal_result_label = Gtk.Label(label="")
        cal_row.pack_start(self.cal_result_label, True, True, 0)
        box.pack_start(cal_row, False, False, 4)

        # --- Motor Test ---
        box.pack_start(self._make_section_label("Motor Test"), False, False, 0)

        speed_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        speed_row.set_margin_start(10)
        speed_row.set_margin_end(10)
        speed_row.pack_start(Gtk.Label(label="Test Speed (m/s):"), False, False, 0)
        self.test_speed_spin = Gtk.SpinButton.new_with_range(0.05, 0.30, 0.01)
        self.test_speed_spin.set_value(0.10)
        self.test_speed_spin.set_digits(2)
        speed_row.pack_start(self.test_speed_spin, False, False, 0)
        box.pack_start(speed_row, False, False, 4)

        btn_grid = Gtk.Grid()
        btn_grid.set_column_spacing(8)
        btn_grid.set_row_spacing(8)
        btn_grid.set_margin_start(10)
        btn_grid.set_margin_end(10)

        motor_tests = [
            ("FORWARD", 0, 0), ("BACKWARD", 0, 1),
            ("LEFT", 1, 0), ("RIGHT", 1, 1),
            ("TURN L", 2, 0), ("TURN R", 2, 1),
        ]
        for label, row, col in motor_tests:
            btn = Gtk.Button(label=label)
            btn.set_size_request(120, 40)
            btn.connect('clicked', self._on_motor_test, label)
            btn_grid.attach(btn, col, row, 1, 1)

        box.pack_start(btn_grid, False, False, 4)

        self.motor_status_label = Gtk.Label(label="")
        self.motor_status_label.set_xalign(0)
        self.motor_status_label.set_margin_start(10)
        box.pack_start(self.motor_status_label, False, False, 4)

        # --- Odometry Reset ---
        box.pack_start(self._make_section_label("Odometry"), False, False, 0)

        odom_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        odom_row.set_margin_start(10)
        reset_btn = Gtk.Button(label="RESET ODOMETRY")
        reset_btn.set_size_request(180, 40)
        reset_btn.connect('clicked', self._on_reset_odom)
        odom_row.pack_start(reset_btn, False, False, 0)
        self.odom_status_label = Gtk.Label(label="")
        odom_row.pack_start(self.odom_status_label, True, True, 0)
        box.pack_start(odom_row, False, False, 4)

        return box

    # --- Calibration handlers ---
    def _on_calibrate(self, button):
        """Read current detection bbox height and update distance calibration."""
        _, detections, _, det_age = state.get_camera()
        if not detections or det_age > 1.0:
            self.cal_result_label.set_text("No detection visible!")
            return

        # Find largest detection (closest pin)
        best = max(detections, key=lambda d: (d['bbox'][3] - d['bbox'][1]))
        x1, y1, x2, y2 = best['bbox']
        box_height = y2 - y1

        if box_height <= 0:
            self.cal_result_label.set_text("Invalid detection bbox")
            return

        known_dist = self.cal_distance_spin.get_value()
        state.set_calibration(float(box_height), known_dist)
        self.cal_result_label.set_text(f"Calibrated: {box_height}px at {known_dist:.1f}m")
        print(f"[Settings] Distance calibration: {box_height}px at {known_dist:.1f}m", flush=True)

    def _on_motor_test(self, button, direction):
        """Run motor in given direction for 1 second."""
        from geometry_msgs.msg import Twist
        node = state._ros_node
        if not node:
            self.motor_status_label.set_text("ROS node not ready")
            return

        speed = self.test_speed_spin.get_value()
        cmd = Twist()

        if direction == "FORWARD":
            cmd.linear.x = speed
        elif direction == "BACKWARD":
            cmd.linear.x = -speed
        elif direction == "LEFT":
            cmd.linear.y = speed
        elif direction == "RIGHT":
            cmd.linear.y = -speed
        elif direction == "TURN L":
            cmd.angular.z = 0.5
        elif direction == "TURN R":
            cmd.angular.z = -0.5

        node.cmd_vel_pub.publish(cmd)
        self.motor_status_label.set_text(f"Testing {direction}...")

        # Cancel previous timer if running
        if self._motor_test_timer:
            GLib.source_remove(self._motor_test_timer)

        def stop_motor():
            stop_cmd = Twist()
            node.cmd_vel_pub.publish(stop_cmd)
            self.motor_status_label.set_text(f"{direction} done")
            self._motor_test_timer = None
            return False  # Don't repeat

        self._motor_test_timer = GLib.timeout_add(1000, stop_motor)

    def _on_reset_odom(self, button):
        """Publish reset command to odometry node."""
        from std_msgs.msg import Empty
        node = state._ros_node
        if not node:
            self.odom_status_label.set_text("ROS node not ready")
            return

        # Create publisher if not exists
        if not hasattr(node, '_reset_odom_pub'):
            node._reset_odom_pub = node.create_publisher(Empty, '/reset_odom', 1)

        node._reset_odom_pub.publish(Empty())
        self.odom_status_label.set_text("Odometry reset sent")
        print("[Settings] Odometry reset", flush=True)


# ============================================================================
# GTK GUI
# ============================================================================

class MainGUI(Gtk.Window):
    """Main fullscreen GUI with map, camera, and control buttons."""

    def __init__(self):
        super().__init__(title="V2N Robot Control")
        self.fullscreen()

        # Main layout
        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.add(vbox)

        # Drawing area for map and camera
        self.da = Gtk.DrawingArea()
        self.da.connect('draw', self.on_draw)
        vbox.pack_start(self.da, True, True, 0)

        # Control panel at bottom
        control_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=20)
        control_box.set_margin_start(20)
        control_box.set_margin_end(20)
        control_box.set_margin_top(10)
        control_box.set_margin_bottom(10)

        # GO button
        self.go_btn = Gtk.Button(label="GO TO TARGET")
        self.go_btn.set_size_request(200, 60)
        self.go_btn.get_style_context().add_class('suggested-action')
        self.go_btn.connect('clicked', self.on_go_clicked)
        control_box.pack_start(self.go_btn, False, False, 0)

        # STOP button
        self.stop_btn = Gtk.Button(label="STOP")
        self.stop_btn.set_size_request(200, 60)
        self.stop_btn.get_style_context().add_class('destructive-action')
        self.stop_btn.connect('clicked', self.on_stop_clicked)
        control_box.pack_start(self.stop_btn, False, False, 0)

        # Status label
        self.status_label = Gtk.Label(label="Status: IDLE")
        self.status_label.set_xalign(0)
        self.status_label.get_style_context().add_class('status-label')
        control_box.pack_start(self.status_label, True, True, 20)

        # Settings button
        self.settings_window = SettingsWindow()
        settings_btn = Gtk.Button(label="SETTINGS")
        settings_btn.set_size_request(120, 60)
        settings_btn.get_style_context().add_class('settings-btn')
        settings_btn.connect('clicked', self.on_settings_clicked)
        control_box.pack_end(settings_btn, False, False, 0)

        # Quit button
        quit_btn = Gtk.Button(label="QUIT")
        quit_btn.set_size_request(100, 60)
        quit_btn.get_style_context().add_class('quit-btn')
        quit_btn.connect('clicked', self.on_quit)
        control_box.pack_end(quit_btn, False, False, 0)

        vbox.pack_start(control_box, False, False, 0)

        # Apply CSS
        self._apply_css()

        # Events
        self.connect('key-press-event', self.on_key)
        self.connect('destroy', self.on_quit)

        # Update timer
        self._timer_id = GLib.timeout_add(33, self.on_tick)

        print("[GUI] Initialized", flush=True)

    def _apply_css(self):
        css = b"""
        window {
            background-color: #0d1117;
        }
        button {
            font-size: 16px;
            font-weight: bold;
            border-radius: 6px;
            padding: 6px 12px;
            border: 1px solid #30363d;
            background-color: #21262d;
            color: #c9d1d9;
        }
        button:hover {
            background-color: #30363d;
            border-color: #8b949e;
        }
        button.suggested-action {
            background: linear-gradient(to bottom, #238636, #1a7f37);
            background-color: #238636;
            color: #ffffff;
            border-color: #238636;
        }
        button.suggested-action:hover {
            background-color: #2ea043;
        }
        button.destructive-action {
            background-color: #da3633;
            color: #ffffff;
            border-color: #da3633;
        }
        button.destructive-action:hover {
            background-color: #f85149;
        }
        button.settings-btn {
            background-color: #1f6feb;
            color: #ffffff;
            border-color: #1f6feb;
        }
        button.settings-btn:hover {
            background-color: #388bfd;
        }
        button.quit-btn {
            background-color: #6e4000;
            color: #ffa657;
            border-color: #6e4000;
        }
        button.quit-btn:hover {
            background-color: #845000;
        }
        label {
            color: #e6edf3;
            font-size: 14px;
        }
        .status-label {
            font-size: 15px;
            font-weight: bold;
            color: #58a6ff;
        }
        notebook {
            background-color: #161b22;
        }
        notebook tab {
            background-color: #21262d;
            color: #c9d1d9;
            padding: 6px 14px;
            border: 1px solid #30363d;
        }
        notebook tab:checked {
            background-color: #0d1117;
            color: #58a6ff;
            border-bottom-color: #58a6ff;
        }
        scale {
            color: #c9d1d9;
        }
        scale trough {
            background-color: #21262d;
            border-radius: 4px;
        }
        scale highlight {
            background-color: #1f6feb;
            border-radius: 4px;
        }
        scale slider {
            background-color: #c9d1d9;
            border-radius: 50%;
        }
        spinbutton {
            background-color: #21262d;
            color: #c9d1d9;
            border: 1px solid #30363d;
            border-radius: 4px;
        }
        """
        style_provider = Gtk.CssProvider()
        style_provider.load_from_data(css)
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(),
            style_provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

    def on_go_clicked(self, button):
        print("[GUI] GO clicked", flush=True)
        state.request_go()

    def on_stop_clicked(self, button):
        print("[GUI] STOP clicked", flush=True)
        state.request_stop()

    def on_settings_clicked(self, button):
        self.settings_window.show_all()
        self.settings_window.present()

    def on_tick(self):
        if not state.running:
            self.on_quit(None)
            return False

        # Update status label
        nav_state, nav_target = state.get_nav_state()
        time_since_target = state.get_time_since_target()
        search_time = state.get_search_time()

        obs_ahead, obs_dist = state.get_obstacle()
        obs_str = f" | OBSTACLE {obs_dist:.2f}m!" if obs_ahead else ""

        if nav_state == "SEARCHING":
            self.status_label.set_text(f"Status: SEARCHING ({search_time:.0f}s) | No target for {time_since_target:.0f}s{obs_str}")
        elif nav_state == "BLIND_APPROACH":
            self.status_label.set_text(f"Status: BLIND APPROACH | Dead-reckoning to pin | Lost {time_since_target:.0f}s{obs_str}")
        elif nav_state == "NAVIGATING" and nav_target:
            self.status_label.set_text(f"Status: NAVIGATING | Target: {nav_target[2]:.2f}m{obs_str}")
        elif nav_state == "ARRIVED":
            self.status_label.set_text(f"Status: ARRIVED at target!")
        else:
            self.status_label.set_text(f"Status: {nav_state} | Press GO to start")

        self.da.queue_draw()
        return True

    def on_key(self, widget, event):
        key = Gdk.keyval_name(event.keyval).lower()
        if key in ('q', 'escape'):
            self.on_quit(None)
        elif key == 'g':
            state.request_go()
        elif key == 's' or key == 'space':
            state.request_stop()
        return True

    def on_draw(self, widget, cr):
        try:
            alloc = widget.get_allocation()
            W, H = alloc.width, alloc.height

            # Background
            cr.set_source_rgb(0.05, 0.07, 0.09)
            cr.paint()

            # Layout
            margin = 10
            title_h = 50
            panel_w = (W - margin * 3) // 2
            panel_h = H - title_h - margin

            map_x, map_y = margin, title_h
            cam_x, cam_y = margin * 2 + panel_w, title_h

            # Title
            cr.set_source_rgb(0.345, 0.651, 1.0)  # #58a6ff
            cr.set_font_size(26)
            cr.move_to(margin, 36)
            cr.show_text("V2N Robot Control")
            # Subtitle
            cr.set_source_rgb(0.55, 0.59, 0.63)  # #8b949e
            cr.set_font_size(14)
            cr.move_to(margin + 280, 36)
            cr.show_text("SLAM + Camera + Navigation")

            # Draw panels
            self.draw_map_panel(cr, map_x, map_y, panel_w, panel_h)
            self.draw_camera_panel(cr, cam_x, cam_y, panel_w, panel_h)

        except Exception as e:
            print(f"[GUI] Draw error: {e}", flush=True)

        return False

    def draw_map_panel(self, cr, x, y, w, h):
        """Draw SLAM map panel."""
        # Panel background
        cr.set_source_rgb(0.086, 0.106, 0.133)  # #161b22
        cr.rectangle(x, y, w, h)
        cr.fill()

        # Border
        cr.set_source_rgb(0.188, 0.212, 0.239)  # #30363d
        cr.set_line_width(1.5)
        cr.rectangle(x, y, w, h)
        cr.stroke()

        # Header
        cr.set_source_rgb(0.345, 0.651, 1.0)  # #58a6ff
        cr.set_font_size(15)
        cr.move_to(x + 10, y + 20)
        cr.show_text("SLAM Map")

        # Get data
        local_map, info, mc = state.get_map()
        rx, ry, rt = state.get_robot_pose()
        lpoints, sc = state.get_laser()

        content_y = y + 35
        content_h = h - 55

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

            # Draw laser points
            cos_t, sin_t = math.cos(rt), math.sin(rt)
            for lx, ly in lpoints:
                wx = rx + lx * cos_t - ly * sin_t
                wy = ry + lx * sin_t + ly * cos_t
                px = int((wx - ox) / res * scale)
                py = int(disp_h - (wy - oy) / res * scale)
                if 0 <= px < disp_w and 0 <= py < disp_h:
                    cv2.circle(disp_map, (px, py), 2, COLOR_LASER, -1)

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

            cr.set_source_rgb(0.55, 0.59, 0.63)  # #8b949e
            cr.set_font_size(11)
            cr.move_to(x + 10, y + h - 10)
            cr.show_text(f"Robot: ({rx:.2f}, {ry:.2f}) {math.degrees(rt):.0f}deg")
        else:
            cr.set_source_rgb(0.35, 0.38, 0.42)
            cr.set_font_size(16)
            cr.move_to(x + w // 2 - 80, y + h // 2)
            cr.show_text("Waiting for map...")

    def draw_camera_panel(self, cr, x, y, w, h):
        """Draw camera panel with detection."""
        cr.set_source_rgb(0.086, 0.106, 0.133)  # #161b22
        cr.rectangle(x, y, w, h)
        cr.fill()

        cr.set_source_rgb(0.188, 0.212, 0.239)  # #30363d
        cr.set_line_width(1.5)
        cr.rectangle(x, y, w, h)
        cr.stroke()

        cr.set_source_rgb(0.902, 0.557, 0.149)  # #e69025 warm amber
        cr.set_font_size(15)
        cr.move_to(x + 10, y + 20)
        cr.show_text("Camera + Detection")

        frame, detections, info, _ = state.get_camera()

        content_y = y + 35
        content_h = h - 55

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
            cr.set_source_rgb(0.35, 0.38, 0.42)
            cr.set_font_size(16)
            cr.move_to(x + w // 2 - 70, y + h // 2)
            cr.show_text("No camera feed")

        cr.set_source_rgb(0.788, 0.820, 0.855)  # #c9d1da
        cr.set_font_size(12)
        cr.move_to(x + 10, y + h - 10)
        cr.show_text(f"Detection: {info}")

    def on_quit(self, widget):
        print("[GUI] Quit requested", flush=True)
        state.request_shutdown()
        if hasattr(self, '_timer_id'):
            GLib.source_remove(self._timer_id)
        Gtk.main_quit()


# ============================================================================
# Main Entry Point
# ============================================================================

_threads = []


def cleanup():
    print("\n[Main] Cleanup started...", flush=True)
    state.request_shutdown()
    for t in _threads:
        if t.is_alive():
            t.join(timeout=2.0)
    print("[Main] Cleanup complete", flush=True)


def signal_handler(sig, frame):
    print(f"\n[Main] Signal {sig} received", flush=True)
    state.request_shutdown()
    try:
        Gtk.main_quit()
    except:
        pass


def main():
    global _threads

    atexit.register(cleanup)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 60, flush=True)
    print("V2N Robot Control - SLAM + Camera + Navigation", flush=True)
    print("=" * 60, flush=True)
    print("Controls:", flush=True)
    print("  GO button / G key  : Navigate to detected target", flush=True)
    print("  STOP button / Space: Emergency stop", flush=True)
    print("  Q / ESC            : Quit", flush=True)
    print("=" * 60, flush=True)

    # Start ROS2 thread
    ros_t = threading.Thread(target=ros_thread, name="ROS2Thread", daemon=True)
    ros_t.start()
    _threads.append(ros_t)

    # Start camera thread
    cam_t = threading.Thread(target=camera_thread, name="CameraThread", daemon=True)
    cam_t.start()
    _threads.append(cam_t)

    time.sleep(1.5)

    if not state.running:
        return

    # Initialize GTK with error check
    ok = Gtk.init_check(sys.argv)[0] if hasattr(Gtk.init_check, '__call__') else True
    if not ok:
        print(f"[Main] GTK init FAILED.", flush=True)
        print(f"[Main]   GDK_BACKEND={os.environ.get('GDK_BACKEND')}", flush=True)
        print(f"[Main]   WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY')}", flush=True)
        print(f"[Main]   XDG_RUNTIME_DIR={os.environ.get('XDG_RUNTIME_DIR')}", flush=True)
        print(f"[Main]   DISPLAY={os.environ.get('DISPLAY')}", flush=True)
        state.request_shutdown()
        return

    try:
        win = MainGUI()
        win.show_all()
        print("[Main] GUI running", flush=True)
        Gtk.main()
    except Exception as e:
        print(f"[Main] GUI error: {e}", flush=True)
        print(f"[Main]   GDK_BACKEND={os.environ.get('GDK_BACKEND')}", flush=True)
        print(f"[Main]   WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY')}", flush=True)
        print(f"[Main]   XDG_RUNTIME_DIR={os.environ.get('XDG_RUNTIME_DIR')}", flush=True)
        print(f"[Main]   DISPLAY={os.environ.get('DISPLAY')}", flush=True)
        state.request_shutdown()


if __name__ == '__main__':
    main()
