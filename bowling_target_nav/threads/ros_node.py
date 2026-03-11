"""ROS2 node for map, scan, TF, and navigation control.

Provides MainGuiNode, the central ROS2 node that:
  - Subscribes to /map (OccupancyGrid) and /scan (LaserScan) for SLAM data.
  - Looks up TF transforms (map->base_link, fallback odom->base_link) to
    get the robot pose.
  - Publishes /cmd_vel (Twist) for navigation, /arduino/cmd (String) for
    direct motor commands, and /reset_odom (Empty) for odometry resets.
  - Broadcasts static TF for camera_link and laser frames based on mounting
    positions configured in the Settings > Setup tab.
  - Runs a 20 Hz control_loop that delegates to the Navigator for target
    tracking, searching, blind approach, and spiral search.
  - Accepts /gui/command (String) messages for remote GO/STOP control.

ros_thread() is the entry point that initializes rclpy, creates the node,
and spins until shutdown.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty, String
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
    """ROS2 node handling sensor subscriptions, TF, and the navigation control loop.

    Subscriptions: /map, /scan, /gui/command.
    Publishers: /cmd_vel, /arduino/cmd, /reset_odom.
    TF: Static broadcaster for camera_link and laser frames.
    Timer: 20 Hz control_loop, 2 Hz sensor TF check.
    """

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
        self.arduino_cmd_pub = self.create_publisher(String, '/arduino/cmd', 1)
        self.reset_odom_pub = self.create_publisher(Empty, '/reset_odom', 1)

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

        # Static TF broadcaster for sensor frames (reads from settings store)
        # Must be static (not dynamic) so the TF is always valid and properly
        # overrides the URDF defaults from robot_state_publisher.
        # A dynamic TF at 2Hz can go stale between publishes, causing
        # Cartographer to fall back to the URDF's identity TF — which
        # garbles the map when lidar_yaw is non-zero.
        self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._last_cam_yaw = None
        self._last_lidar_yaw = None
        self._last_cam_pos = None
        self._last_lidar_pos = None
        self.create_timer(0.5, self._publish_sensor_tfs)  # check for changes at 2Hz

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Main GUI ROS node started')

    def _update_target_preview(self, target):
        """Update map-frame target position and LiDAR comparison for GUI.

        Called every control loop cycle (even when idle) so the target
        diamond appears on the map in real-time and the camera panel can
        show LiDAR distance for comparison with vision distance.
        """
        cam_angle = -target.get('angle', 0.0)  # camera→ROS convention
        camera_yaw_rad = math.radians(state.detection.get_camera_yaw())
        angle = cam_angle + camera_yaw_rad  # base_link frame
        vision_distance = target.get('distance', 1.0)

        # LiDAR distance at target angle for GUI comparison
        lidar_angle = self.navigator._base_to_laser_angle(angle)
        lidar_dist = state.sensors.get_lidar_distance_at_angle(lidar_angle)
        state.nav.set_lidar_at_target(lidar_dist)

        # Map-frame target for GUI overlay via TF2
        try:
            from geometry_msgs.msg import PointStamped
            from tf2_geometry_msgs import do_transform_point
            pt = PointStamped()
            pt.header.stamp = rclpy.time.Time().to_msg()
            pt.header.frame_id = 'camera_link'
            pt.point.x = vision_distance * math.cos(cam_angle)
            pt.point.y = vision_distance * math.sin(cam_angle)
            pt.point.z = 0.0
            tf = self.tf_buffer.lookup_transform('map', 'camera_link',
                                                  rclpy.time.Time())
            map_pt = do_transform_point(pt, tf)
            state.nav.set_nav_target_map(map_pt.point.x, map_pt.point.y)
        except Exception:
            # Fallback: manual computation
            try:
                robot_x, robot_y, robot_theta = state.sensors.get_robot_pose()
                cam_pos = state.detection.get_camera_pos()
                det_x = vision_distance * math.cos(cam_angle)
                det_y = vision_distance * math.sin(cam_angle)
                cos_cy = math.cos(camera_yaw_rad)
                sin_cy = math.sin(camera_yaw_rad)
                body_x = cam_pos['x'] + det_x * cos_cy - det_y * sin_cy
                body_y = cam_pos['y'] + det_x * sin_cy + det_y * cos_cy
                cos_t = math.cos(robot_theta)
                sin_t = math.sin(robot_theta)
                map_tx = robot_x + body_x * cos_t - body_y * sin_t
                map_ty = robot_y + body_x * sin_t + body_y * cos_t
                state.nav.set_nav_target_map(map_tx, map_ty)
            except Exception:
                pass

    def _publish_sensor_tfs(self):
        """Publish static TF frames for camera and LiDAR from settings store.

        Uses StaticTransformBroadcaster so TFs are always valid (no staleness).
        Only republishes when settings actually change.
        """
        cam = state.detection.get_camera_pos()
        cam_yaw = state.detection.get_camera_yaw()
        lidar = state.detection.get_lidar_pos()
        lidar_yaw_deg = state.detection.get_lidar_yaw()

        # Skip if nothing changed
        if (cam_yaw == self._last_cam_yaw and
                lidar_yaw_deg == self._last_lidar_yaw and
                cam == self._last_cam_pos and
                lidar == self._last_lidar_pos):
            return
        self._last_cam_yaw = cam_yaw
        self._last_lidar_yaw = lidar_yaw_deg
        self._last_cam_pos = dict(cam) if cam else None
        self._last_lidar_pos = dict(lidar) if lidar else None

        now = self.get_clock().now().to_msg()

        # Camera frame
        cam_yaw_rad = math.radians(cam_yaw)
        t_cam = TransformStamped()
        t_cam.header.stamp = now
        t_cam.header.frame_id = 'base_link'
        t_cam.child_frame_id = 'camera_link'
        t_cam.transform.translation.x = cam['x']
        t_cam.transform.translation.y = cam['y']
        t_cam.transform.translation.z = cam['z']
        t_cam.transform.rotation.x = 0.0
        t_cam.transform.rotation.y = 0.0
        t_cam.transform.rotation.z = math.sin(cam_yaw_rad / 2.0)
        t_cam.transform.rotation.w = math.cos(cam_yaw_rad / 2.0)

        # LiDAR frame (with mounting yaw — e.g. LiDAR mounted at 180°)
        lidar_yaw = math.radians(lidar_yaw_deg)
        t_lidar = TransformStamped()
        t_lidar.header.stamp = now
        t_lidar.header.frame_id = 'base_link'
        t_lidar.child_frame_id = 'laser'
        t_lidar.transform.translation.x = lidar['x']
        t_lidar.transform.translation.y = lidar['y']
        t_lidar.transform.translation.z = lidar['z']
        t_lidar.transform.rotation.x = 0.0
        t_lidar.transform.rotation.y = 0.0
        t_lidar.transform.rotation.z = math.sin(lidar_yaw / 2.0)
        t_lidar.transform.rotation.w = math.cos(lidar_yaw / 2.0)

        self._tf_broadcaster.sendTransform([t_cam, t_lidar])

    def map_cb(self, msg):
        """Process /map OccupancyGrid: colorize cells and store in shared state."""
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
        """Process /scan LaserScan: convert to cartesian points, look up robot pose via TF."""
        if not state.running:
            return
        try:
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
                x_l = r_v * np.cos(a_v)
                y_l = r_v * np.sin(a_v)
                # Rotate laser points from laser frame to base_link frame
                lidar_yaw = math.radians(state.detection.get_lidar_yaw())
                if lidar_yaw != 0.0:
                    cos_ly = math.cos(lidar_yaw)
                    sin_ly = math.sin(lidar_yaw)
                    x_b = x_l * cos_ly - y_l * sin_ly
                    y_b = x_l * sin_ly + y_l * cos_ly
                    points = np.column_stack((x_b, y_b))
                else:
                    points = np.column_stack((x_l, y_l))
            else:
                points = np.empty((0, 2), dtype=np.float32)
            state.sensors.set_laser(points)
            state.sensors.set_raw_scan(msg)

            # Pose lookup: try SLAM (map->base_link), fall back to odometry
            t = None
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            except TransformException:
                try:
                    t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                except TransformException:
                    pass
            if t is not None:
                x = t.transform.translation.x
                y = t.transform.translation.y
                q = t.transform.rotation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                theta = math.atan2(siny, cosy)
                state.sensors.set_robot_pose(x, y, theta)
        except Exception as e:
            state.add_error('ros_scan', str(e))

    def command_cb(self, msg):
        """Handle /gui/command messages for remote GO/STOP control."""
        cmd = msg.data.strip().lower()
        if cmd == 'go':
            state.nav.request_go()
        elif cmd == 'stop':
            state.nav.request_stop()

    def control_loop(self):
        """Main 20 Hz control loop -- delegates all navigation to Navigator.

        State machine flow:
          1. Check for STOP request -> halt and go IDLE.
          2. Check for GO request -> activate navigation.
          3. If target visible -> navigate_to_target (or handle ARRIVED).
          4. If target lost briefly -> drift forward at min speed.
          5. If target lost beyond timeout -> blind approach (if close) or search.
          6. SEARCHING -> 360-degree rotation scan, then spiral if enabled.
          7. SPIRAL_SEARCH -> expanding spiral, give up if exhausted.
        """
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
            nav._arrival_first_seen = 0.0  # Reset stale arrival timer
            state.detection.request_bbox_reset()  # Reset bbox size tracker
            state.nav.set_nav_state("IDLE", None)
            self.get_logger().info("Navigation activated")

        # Always update target preview on map + LiDAR comparison (even in idle)
        _, detections_preview, _, det_age_preview = state.detection.get_camera()
        if det_age_preview <= state.detection.get_detect_expiry():
            preview_target = find_best_target(detections_preview)
        else:
            preview_target = None
        if preview_target:
            self._update_target_preview(preview_target)
        elif not self.is_active:
            state.nav.clear_nav_target_map()
            state.nav.set_lidar_at_target(float('inf'))

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

            if nav_state in ("SEARCHING", "BLIND_APPROACH", "SPIRAL_SEARCH"):
                if nav._spiral_active:
                    nav._spiral_active = False
                    nav._spiral_radius = 0.0
                    nav._spiral_start_time = 0.0
                nav._search_initialized = False
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
                # Check obstacles first — don't drift into walls
                min_front, _, _ = nav.check_obstacles()
                if min_front < nav.obstacle_distance:
                    # Obstacle ahead — stop drifting, enter blind approach or search
                    nav.stop_robot()
                else:
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
                # Safety timeout (odometry might be broken)
                if search_time > nav.search_timeout:
                    self.get_logger().warn(f"Search safety timeout ({search_time:.1f}s)")
                    nav.stop_robot()
                    self.is_active = False
                    state.nav.set_nav_state("IDLE", None)
                elif nav.search_rotate():  # Returns True when 360° complete
                    if nav.spiral_search_enabled:
                        self.get_logger().info("360° scan done - starting spiral search")
                        nav.start_spiral_search()
                    else:
                        self.get_logger().warn("360° scan complete - target not found")
                        self.is_active = False
                        state.nav.set_nav_state("IDLE", None)

            elif nav_state == "SPIRAL_SEARCH":
                if nav.spiral_search_step():
                    self.get_logger().warn("Spiral search exhausted - giving up")
                    nav.stop_robot()
                    self.is_active = False
                    state.nav.set_nav_state("IDLE", None)

            elif nav_state == "IDLE" and self.is_active:
                self.get_logger().info("No target visible - searching...")
                state.nav.start_search()
                nav.search_rotate()


def ros_thread():
    """ROS2 spin thread entry point -- init rclpy, create node, spin until shutdown."""
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
                    import traceback
                    print(f"[ROS Thread] spin_once error: {e}", flush=True)
                    traceback.print_exc()
                    state.add_error('ros_spin', str(e))
                break
    except Exception as e:
        import traceback
        print(f"[ROS Thread] Fatal error: {e}", flush=True)
        traceback.print_exc()
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
