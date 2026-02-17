#!/usr/bin/env python3
"""
Target Follower Node with LiDAR+Vision Distance Fusion
=======================================================

Receives target pose from vision node and navigates the robot toward
the target while avoiding obstacles. Fuses LiDAR range data with
vision-estimated distance for robust positioning.

Distance Fusion Strategy:
    1. Vision provides target angle (reliable) and bbox-based distance (noisy)
    2. LiDAR provides precise range at any angle
    3. When LiDAR has a return at the vision-detected angle, use LiDAR distance
    4. Fall back to vision distance estimate when LiDAR has no return

Subscriptions:
    /target_pose (PoseStamped) - Target position from vision node
    /scan (LaserScan) - LiDAR for obstacle detection AND distance fusion

Publications:
    /cmd_vel (Twist) - Robot velocity commands (direct mode)

Parameters:
    mode: "nav2" or "direct"
    approach_distance: Distance to stop from target (default 0.3m)
    linear_speed: Max linear velocity (default 0.15 m/s)
    angular_speed: Max angular velocity (default 0.3 rad/s)
    lost_timeout: Seconds before searching (default 2.0)
    obstacle_distance: Min distance to obstacles (default 0.25m)
    lidar_fusion_window: Angular window for LiDAR lookup (default 0.15 rad ~8.6 deg)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped


class TargetFollowerNode(Node):
    """Target follower with LiDAR+Vision distance fusion."""

    def __init__(self):
        super().__init__('target_follower_node')

        # Parameters
        self.declare_parameter('mode', 'direct')
        self.declare_parameter('approach_distance', 0.3)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('lost_timeout', 2.0)
        self.declare_parameter('search_angular_vel', 0.2)
        self.declare_parameter('obstacle_distance', 0.25)
        self.declare_parameter('angle_tolerance', 0.1)
        self.declare_parameter('goal_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('enabled', True)
        self.declare_parameter('lidar_fusion_window', 0.15)
        # Note: use_sim_time is auto-declared by ROS2 Node base class

        self.mode = self.get_parameter('mode').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.search_angular_vel = self.get_parameter('search_angular_vel').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.enabled = self.get_parameter('enabled').value
        self.lidar_fusion_window = self.get_parameter('lidar_fusion_window').value

        # State
        self.last_target_time = 0.0
        self.last_target_pose = None
        self.current_goal_handle = None
        self.min_front_distance = float('inf')
        self.state = 'IDLE'

        # LiDAR data for fusion
        self._last_scan = None
        self._last_scan_time = 0.0

        # Last known target in odom frame (for memory after lost)
        self._last_known_target_odom = None

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.target_sub = self.create_subscription(
            PoseStamped, '/target_pose', self.target_callback, qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Nav2 action client
        if self.mode == 'nav2':
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.get_logger().info("Waiting for Nav2 action server...")
            self.nav_client.wait_for_server(timeout_sec=5.0)

        # Control loop at 10Hz
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"Target follower started: mode={self.mode}, "
            f"lidar_fusion_window={math.degrees(self.lidar_fusion_window):.1f}deg"
        )

    def target_callback(self, msg: PoseStamped):
        """Handle incoming target pose from vision node."""
        self.last_target_time = time.time()
        self.last_target_pose = msg

        if self.state in ('SEARCHING', 'IDLE'):
            self.state = 'TRACKING'
            self.get_logger().info("Target acquired, tracking...")

    def scan_callback(self, msg: LaserScan):
        """Store full LiDAR scan for both obstacle detection and distance fusion."""
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        self._last_scan = msg
        self._last_scan_time = time.time()

        # Obstacle detection: front sector (-30 to +30 degrees)
        front_angle = 0.5  # radians
        start_idx = int((-front_angle - msg.angle_min) / msg.angle_increment)
        end_idx = int((front_angle - msg.angle_min) / msg.angle_increment)
        start_idx = max(0, min(num_ranges - 1, start_idx))
        end_idx = max(0, min(num_ranges - 1, end_idx))

        front_ranges = msg.ranges[start_idx:end_idx + 1]
        valid_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]
        self.min_front_distance = min(valid_ranges) if valid_ranges else float('inf')

    def _get_lidar_distance_at_angle(self, angle_rad: float) -> float:
        """Look up LiDAR range at a specific angle for distance fusion.

        Searches a small angular window around the target angle and
        returns the minimum valid range (closest object at that bearing).

        Args:
            angle_rad: Target angle in radians (0=forward, +=left, -=right)

        Returns:
            Distance in meters, or inf if no valid LiDAR return
        """
        scan = self._last_scan
        if scan is None:
            return float('inf')

        # Check scan freshness (must be < 0.5s old)
        if time.time() - self._last_scan_time > 0.5:
            return float('inf')

        # Convert target angle to LiDAR scan index range
        half_window = self.lidar_fusion_window
        angle_min = angle_rad - half_window
        angle_max = angle_rad + half_window

        start_idx = int((angle_min - scan.angle_min) / scan.angle_increment)
        end_idx = int((angle_max - scan.angle_min) / scan.angle_increment)

        num_ranges = len(scan.ranges)
        start_idx = max(0, min(num_ranges - 1, start_idx))
        end_idx = max(0, min(num_ranges - 1, end_idx))

        if start_idx > end_idx:
            return float('inf')

        # Find minimum valid range in the window
        valid = [
            r for r in scan.ranges[start_idx:end_idx + 1]
            if scan.range_min < r < scan.range_max
        ]

        return min(valid) if valid else float('inf')

    def control_loop(self):
        """Main control loop at 10Hz."""
        if not self.enabled:
            return

        time_since_target = time.time() - self.last_target_time

        # Check if target is lost
        if time_since_target > self.lost_timeout:
            if self.state in ('TRACKING', 'APPROACHING'):
                self.state = 'SEARCHING'
                self.get_logger().info("Target lost, searching...")

        if self.state == 'IDLE':
            self._stop()
        elif self.state == 'SEARCHING':
            self._search()
        elif self.state == 'TRACKING':
            self._track()
        elif self.state == 'APPROACHING':
            self._approach()

    def _stop(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def _search(self):
        cmd = Twist()
        cmd.angular.z = self.search_angular_vel
        self.cmd_vel_pub.publish(cmd)

    def _track(self):
        if self.last_target_pose is None:
            return

        if self.mode == 'nav2':
            self._track_nav2()
        else:
            self._track_direct()

    def _track_direct(self):
        """Direct velocity control with LiDAR+Vision distance fusion."""
        pose = self.last_target_pose

        target_x = pose.pose.position.x
        target_y = pose.pose.position.y

        # Vision-based distance and angle
        vision_distance = math.sqrt(target_x ** 2 + target_y ** 2)
        angle = math.atan2(target_y, target_x)

        # LiDAR+Vision fusion: use LiDAR distance at the vision-detected angle
        lidar_distance = self._get_lidar_distance_at_angle(angle)

        if lidar_distance < float('inf'):
            # LiDAR has a return - use it (more precise than bbox estimation)
            distance = lidar_distance
            source = "lidar"
        else:
            # No LiDAR return - fall back to vision estimate
            distance = vision_distance
            source = "vision"

        cmd = Twist()

        # Obstacle check
        if self.min_front_distance < self.obstacle_distance:
            self.get_logger().warn(
                f"Obstacle at {self.min_front_distance:.2f}m, stopping",
                throttle_duration_sec=2.0
            )
            self._stop()
            return

        # Rotate to face target
        if abs(angle) > self.angle_tolerance:
            cmd.angular.z = self.angular_speed * (1.0 if angle > 0 else -1.0)
            cmd.angular.z *= min(1.0, abs(angle) / 0.5)
        else:
            # Move forward if aligned
            if distance > self.approach_distance:
                speed_factor = min(1.0, (distance - self.approach_distance) / 0.5)
                cmd.linear.x = self.linear_speed * speed_factor
                cmd.angular.z = self.angular_speed * 0.5 * (angle / 0.5)
            else:
                self.state = 'APPROACHING'
                self.get_logger().info(
                    f"Reached target at {distance:.2f}m (source={source})"
                )

        self.cmd_vel_pub.publish(cmd)

    def _track_nav2(self):
        """Send goal to Nav2 with fused distance."""
        if self.current_goal_handle is not None:
            return

        pose = self.last_target_pose

        try:
            if pose.header.frame_id != self.goal_frame:
                transform = self.tf_buffer.lookup_transform(
                    self.goal_frame, pose.header.frame_id, rclpy.time.Time()
                )
                pose = do_transform_pose_stamped(pose, transform)
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")
            return

        target_x = pose.pose.position.x
        target_y = pose.pose.position.y

        try:
            robot_tf = self.tf_buffer.lookup_transform(
                self.goal_frame, self.robot_frame, rclpy.time.Time()
            )
            robot_x = robot_tf.transform.translation.x
            robot_y = robot_tf.transform.translation.y
        except Exception:
            robot_x, robot_y = 0.0, 0.0

        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.sqrt(dx ** 2 + dy ** 2)

        if dist < self.approach_distance:
            self.state = 'APPROACHING'
            return

        ux, uy = dx / dist, dy / dist
        goal_x = target_x - self.approach_distance * ux
        goal_y = target_y - self.approach_distance * uy
        yaw = math.atan2(target_y - goal_y, target_x - goal_x)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(f"Nav2 goal: ({goal_x:.2f}, {goal_y:.2f})")
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected")
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        self.current_goal_handle = None
        result = future.result()
        self.get_logger().info(f"Nav2 goal completed: {result.status}")

    def _approach(self):
        """Final approach - stop and re-track if target still visible."""
        self._stop()
        if time.time() - self.last_target_time < 0.5:
            self.state = 'TRACKING'


def main(args=None):
    rclpy.init(args=args)
    node = TargetFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
