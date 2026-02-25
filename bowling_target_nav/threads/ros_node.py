"""ROS2 node for map, scan, TF, and navigation control."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException
import numpy as np
import cv2

try:
    from nav2_msgs.action import NavigateToPose
    HAS_NAV2 = True
except ImportError:
    HAS_NAV2 = False

from bowling_target_nav.state import state
from bowling_target_nav.nav.navigator import Navigator
from bowling_target_nav.nav.target_selector import find_best_target

# Colors (BGR for OpenCV map rendering)
COLOR_UNKNOWN = (30, 30, 30)
COLOR_FREE = (50, 50, 50)
COLOR_OCCUPIED = (0, 200, 255)


class MainGuiNode(Node):
    """ROS2 node handling sensors, TF, and navigation control loop."""

    def __init__(self):
        super().__init__('main_gui_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, sensor_qos)
        self.create_subscription(String, '/gui/command', self.command_cb, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        nav_client = None
        if HAS_NAV2:
            nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create navigator (owns all navigation algorithms)
        self.navigator = Navigator(
            state=state,
            cmd_vel_pub=self.cmd_vel_pub,
            tf_buffer=self.tf_buffer,
            nav_client=nav_client,
            logger=self.get_logger())

        self.is_active = False
        self.last_target = None

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Main GUI ROS node started')

    def map_cb(self, msg):
        if not state.running:
            return
        try:
            w, h = msg.info.width, msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))
            img = np.zeros((h, w, 3), dtype=np.uint8)
            img[data == -1] = COLOR_UNKNOWN
            img[data == 0] = COLOR_FREE
            img[data > 0] = COLOR_OCCUPIED
            state.sensors.set_map(cv2.flip(img, 0), msg.info)
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
            state.sensors.set_laser(points)
            state.sensors.set_raw_scan(msg)

            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                x = t.transform.translation.x
                y = t.transform.translation.y
                q = t.transform.rotation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                theta = math.atan2(siny, cosy)
                state.sensors.set_robot_pose(x, y, theta)
            except TransformException:
                pass
        except Exception as e:
            state.add_error('ros_scan', str(e))

    def command_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'go':
            state.nav.request_go()
        elif cmd == 'stop':
            state.nav.request_stop()

    def control_loop(self):
        """Main 20Hz control loop - delegates navigation to Navigator."""
        nav = self.navigator

        if state.nav.check_and_clear_stop():
            nav.stop_robot()
            self.is_active = False
            nav.blind_approach_active = False
            state.nav.set_nav_state("IDLE")
            state.nav.clear_nav_target_map()
            self.get_logger().info("Navigation stopped")
            return

        if state.nav.check_and_clear_go():
            self.is_active = True
            nav.blind_approach_active = False
            state.nav.set_nav_state("IDLE", None)
            self.get_logger().info("Navigation activated")

        if not self.is_active:
            return

        _, detections, _, det_age = state.detection.get_camera()
        detect_expiry = state.detection.get_detect_expiry()
        if det_age > detect_expiry:
            detections = []

        target = find_best_target(detections)
        nav_state, _ = state.nav.get_nav_state()

        if target:
            state.nav.update_target_seen()
            self.last_target = target

            if nav.blind_approach_active:
                self.get_logger().info("Target re-detected during BLIND_APPROACH")
                nav.blind_approach_active = False
                state.nav.set_nav_state("NAVIGATING")

            if nav_state in ("SEARCHING", "BLIND_APPROACH"):
                self.get_logger().info("Target found!")

            # ARRIVED is a terminal state - user must press GO again
            if nav_state == "ARRIVED":
                nav.stop_robot()
                self.is_active = False
                return

            nav.navigate_to_target(target)
        else:
            time_since = state.nav.get_time_since_target()

            if nav.blind_approach_active:
                nav.blind_approach_step()
                return

            if nav_state == "NAVIGATING" and time_since <= nav.lost_timeout and self.last_target:
                # Drift forward at minimum speed while target briefly lost
                cmd = Twist()
                cmd.linear.x = nav.min_linear_speed
                nav._publish_cmd(cmd)
                return

            if nav_state == "NAVIGATING" and time_since > nav.lost_timeout:
                _, nav_target = state.nav.get_nav_state()
                last_dist = nav_target[2] if nav_target else float('inf')

                if last_dist < nav.blind_approach_entry_distance:
                    if nav.enter_blind_approach():
                        nav.blind_approach_step()
                        return

                self.get_logger().warn(f"Target lost for {time_since:.1f}s - searching...")
                state.nav.start_search()
                nav.search_rotate()

            elif nav_state == "SEARCHING":
                search_time = state.nav.get_search_time()
                if search_time > nav.search_timeout:
                    self.get_logger().warn(f"Search timeout ({search_time:.1f}s)")
                    nav.stop_robot()
                    self.is_active = False
                    state.nav.set_nav_state("IDLE", None)
                else:
                    nav.search_rotate()

            elif nav_state == "IDLE" and self.is_active:
                self.get_logger().info("No target visible - searching...")
                state.nav.start_search()
                nav.search_rotate()


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
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print("[ROS Thread] Stopped", flush=True)
