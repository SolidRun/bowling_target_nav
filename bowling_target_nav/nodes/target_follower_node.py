#!/usr/bin/env python3
"""
Target Follower Node for Bowling Target Navigation
===================================================

ROS2 node that receives target pose from vision node and navigates
the robot toward the target while avoiding obstacles.

Modes of operation:
    1. NAV2_MODE: Sends goals to Nav2 for path planning with obstacle avoidance
    2. DIRECT_MODE: Direct velocity control (simpler, no Nav2 required)

Subscriptions:
    /target_pose (geometry_msgs/PoseStamped) - Target position from vision node
    /scan (sensor_msgs/LaserScan) - LiDAR for obstacle detection (direct mode)

Publications:
    /cmd_vel (geometry_msgs/Twist) - Robot velocity commands (direct mode)

Actions:
    /navigate_to_pose - Nav2 navigation action (Nav2 mode)

Parameters:
    mode: "nav2" or "direct"
    approach_distance: Distance to stop from target (default 0.3m)
    linear_speed: Maximum linear velocity (default 0.15 m/s)
    angular_speed: Maximum angular velocity (default 0.3 rad/s)
    lost_timeout: Seconds before searching if no target (default 2.0)
    search_angular_vel: Angular velocity while searching (default 0.2 rad/s)
    obstacle_distance: Minimum distance to obstacles (default 0.25m)
    angle_tolerance: Angle error to consider "centered" (default 0.1 rad)

Usage:
    ros2 run bowling_target_nav target_follower_node
    ros2 run bowling_target_nav target_follower_node --ros-args -p mode:=direct
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
    """
    ROS2 node for following detected targets.

    Receives target pose from vision node and controls robot motion
    to approach the target while avoiding obstacles.
    """

    def __init__(self):
        super().__init__('target_follower_node')

        # Declare parameters
        self.declare_parameter('mode', 'direct')  # 'nav2' or 'direct'
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

        # Get parameters
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

        # State
        self.last_target_time = 0.0
        self.last_target_pose = None
        self.current_goal_handle = None
        self.min_front_distance = float('inf')
        self.state = 'IDLE'  # IDLE, TRACKING, SEARCHING, APPROACHING

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

        # Nav2 action client (if using Nav2 mode)
        if self.mode == 'nav2':
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.get_logger().info("Waiting for Nav2 action server...")
            self.nav_client.wait_for_server(timeout_sec=5.0)

        # Control loop timer
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f"Target follower started: mode={self.mode}")

    def target_callback(self, msg: PoseStamped):
        """Handle incoming target pose from vision node."""
        self.last_target_time = time.time()
        self.last_target_pose = msg

        if self.state == 'SEARCHING' or self.state == 'IDLE':
            self.state = 'TRACKING'
            self.get_logger().info("Target acquired, tracking...")

    def scan_callback(self, msg: LaserScan):
        """Process LiDAR scan for obstacle detection."""
        # Check front sector (roughly -30 to +30 degrees)
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Calculate indices for front sector
        front_angle = 0.5  # radians (~30 degrees)
        start_idx = int(((-front_angle - msg.angle_min) / msg.angle_increment))
        end_idx = int(((front_angle - msg.angle_min) / msg.angle_increment))

        start_idx = max(0, min(num_ranges - 1, start_idx))
        end_idx = max(0, min(num_ranges - 1, end_idx))

        # Get minimum distance in front
        front_ranges = msg.ranges[start_idx:end_idx + 1]
        valid_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]

        if valid_ranges:
            self.min_front_distance = min(valid_ranges)
        else:
            self.min_front_distance = float('inf')

    def control_loop(self):
        """Main control loop - runs at 10Hz."""
        if not self.enabled:
            return

        current_time = time.time()
        time_since_target = current_time - self.last_target_time

        # Check if target is lost
        if time_since_target > self.lost_timeout:
            if self.state == 'TRACKING' or self.state == 'APPROACHING':
                self.state = 'SEARCHING'
                self.get_logger().info("Target lost, searching...")

        # State machine
        if self.state == 'IDLE':
            self._stop()

        elif self.state == 'SEARCHING':
            self._search()

        elif self.state == 'TRACKING':
            self._track()

        elif self.state == 'APPROACHING':
            self._approach()

    def _stop(self):
        """Stop all motion."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def _search(self):
        """Rotate in place to search for target."""
        cmd = Twist()
        cmd.angular.z = self.search_angular_vel
        self.cmd_vel_pub.publish(cmd)

    def _track(self):
        """Track and approach detected target."""
        if self.last_target_pose is None:
            return

        if self.mode == 'nav2':
            self._track_nav2()
        else:
            self._track_direct()

    def _track_direct(self):
        """Direct velocity control to follow target."""
        pose = self.last_target_pose

        # Target is in base_link frame from vision node
        target_x = pose.pose.position.x
        target_y = pose.pose.position.y

        distance = math.sqrt(target_x ** 2 + target_y ** 2)
        angle = math.atan2(target_y, target_x)

        cmd = Twist()

        # Check for obstacles
        if self.min_front_distance < self.obstacle_distance:
            self.get_logger().warn(f"Obstacle at {self.min_front_distance:.2f}m, stopping")
            self._stop()
            return

        # First, rotate to face target
        if abs(angle) > self.angle_tolerance:
            # Rotate toward target
            cmd.angular.z = self.angular_speed * (1.0 if angle > 0 else -1.0)
            # Slow rotation as we approach alignment
            cmd.angular.z *= min(1.0, abs(angle) / 0.5)
        else:
            # Move forward if aligned and not too close
            if distance > self.approach_distance:
                # Speed proportional to distance (slow down as we approach)
                speed_factor = min(1.0, (distance - self.approach_distance) / 0.5)
                cmd.linear.x = self.linear_speed * speed_factor

                # Small angular correction while moving
                cmd.angular.z = self.angular_speed * 0.5 * (angle / 0.5)
            else:
                # Arrived at target
                self.state = 'APPROACHING'
                self.get_logger().info(f"Reached target at {distance:.2f}m")

        self.cmd_vel_pub.publish(cmd)

    def _track_nav2(self):
        """Send goal to Nav2 for path planning."""
        if self.current_goal_handle is not None:
            # Already navigating
            return

        pose = self.last_target_pose

        # Transform to goal frame if needed
        try:
            if pose.header.frame_id != self.goal_frame:
                transform = self.tf_buffer.lookup_transform(
                    self.goal_frame,
                    pose.header.frame_id,
                    rclpy.time.Time()
                )
                pose = do_transform_pose_stamped(pose, transform)
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")
            return

        # Compute approach goal (stop approach_distance before target)
        target_x = pose.pose.position.x
        target_y = pose.pose.position.y

        # Get robot position in goal frame
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                self.goal_frame, self.robot_frame, rclpy.time.Time()
            )
            robot_x = robot_tf.transform.translation.x
            robot_y = robot_tf.transform.translation.y
        except Exception:
            robot_x, robot_y = 0.0, 0.0

        # Vector from robot to target
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.sqrt(dx ** 2 + dy ** 2)

        if dist < self.approach_distance:
            self.state = 'APPROACHING'
            return

        # Unit vector
        ux, uy = dx / dist, dy / dist

        # Goal position: approach_distance before target
        goal_x = target_x - self.approach_distance * ux
        goal_y = target_y - self.approach_distance * uy

        # Face the target
        yaw = math.atan2(target_y - goal_y, target_x - goal_x)

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

        # Send goal
        self.get_logger().info(f"Sending Nav2 goal: ({goal_x:.2f}, {goal_y:.2f})")
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handle Nav2 goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected")
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Handle Nav2 goal completion."""
        self.current_goal_handle = None
        result = future.result()
        self.get_logger().info(f"Nav2 goal completed: {result.status}")

    def _approach(self):
        """Final approach behavior when close to target."""
        # Could implement fine positioning here
        # For now, just stop
        self._stop()

        # Go back to tracking if we still see target
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
        # Stop robot
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
