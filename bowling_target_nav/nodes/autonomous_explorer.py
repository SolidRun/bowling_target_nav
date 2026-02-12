#!/usr/bin/env python3
"""
Autonomous Explorer Node
========================

Scans for bowling targets, approaches and pushes them.

State machine:
    SCAN_ROTATE -> Rotate 360° in place, looking for target with camera
    WANDER      -> No target found after scan, drive around exploring
    APPROACH    -> Target detected! Steer toward it
    PUSH        -> Close enough, drive forward through target
    BACKUP      -> After push, reverse then scan again

Subscriptions:
    /target_pose (PoseStamped) - from vision_node
    /scan (LaserScan) - from LiDAR for obstacle avoidance

Publications:
    /cmd_vel (Twist) - robot velocity commands

Usage:
    ros2 run bowling_target_nav autonomous_explorer
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan


class AutonomousExplorer(Node):

    # States
    SCAN_ROTATE = 'SCAN_ROTATE'
    WANDER = 'WANDER'
    APPROACH = 'APPROACH'
    PUSH = 'PUSH'
    BACKUP = 'BACKUP'
    AVOID_TURN = 'AVOID_TURN'

    def __init__(self):
        super().__init__('autonomous_explorer')

        # Parameters
        self.declare_parameter('scan_angular_speed', 0.35)    # rad/s rotation during scan
        self.declare_parameter('linear_speed', 0.18)          # m/s wander speed
        self.declare_parameter('angular_speed', 0.4)          # rad/s turn speed
        self.declare_parameter('approach_linear', 0.15)       # m/s approach speed
        self.declare_parameter('approach_angular', 0.35)      # rad/s approach steering
        self.declare_parameter('push_speed', 0.20)            # m/s push-through speed
        self.declare_parameter('push_duration', 2.5)          # seconds to push
        self.declare_parameter('backup_speed', -0.12)         # m/s backup speed
        self.declare_parameter('backup_duration', 1.5)        # seconds to backup
        self.declare_parameter('obstacle_distance', 0.25)     # m - stop if closer
        self.declare_parameter('obstacle_slow_distance', 0.40)  # m - slow down
        self.declare_parameter('turn_duration_min', 1.0)      # s min random turn
        self.declare_parameter('turn_duration_max', 2.5)      # s max random turn
        self.declare_parameter('target_lost_timeout', 1.5)    # s before giving up target
        self.declare_parameter('approach_distance', 0.35)     # m - switch to PUSH
        self.declare_parameter('angle_tolerance', 0.15)       # rad - "centered enough"
        self.declare_parameter('control_rate', 10.0)          # Hz
        self.declare_parameter('wander_before_rescan', 8.0)   # s of wandering before scanning again

        # Read params
        self.scan_angular_speed = self.get_parameter('scan_angular_speed').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.approach_linear = self.get_parameter('approach_linear').value
        self.approach_angular = self.get_parameter('approach_angular').value
        self.push_speed = self.get_parameter('push_speed').value
        self.push_duration = self.get_parameter('push_duration').value
        self.backup_speed = self.get_parameter('backup_speed').value
        self.backup_duration = self.get_parameter('backup_duration').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.obstacle_slow_distance = self.get_parameter('obstacle_slow_distance').value
        self.turn_duration_min = self.get_parameter('turn_duration_min').value
        self.turn_duration_max = self.get_parameter('turn_duration_max').value
        self.target_lost_timeout = self.get_parameter('target_lost_timeout').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.control_rate = self.get_parameter('control_rate').value
        self.wander_before_rescan = self.get_parameter('wander_before_rescan').value

        # Compute scan duration for full 360°
        # 2*pi / angular_speed = time for full rotation
        self.scan_duration = (2.0 * math.pi) / self.scan_angular_speed

        # State
        self.state = self.SCAN_ROTATE
        self.state_start_time = time.time()
        self.scan_end_time = time.time() + self.scan_duration
        self.turn_end_time = 0.0
        self.turn_direction = 1.0
        self.push_end_time = 0.0
        self.backup_end_time = 0.0
        self.wander_start_time = 0.0
        self.targets_pushed = 0

        # LiDAR data
        self.min_front_dist = float('inf')
        self.min_left_dist = float('inf')
        self.min_right_dist = float('inf')

        # Target tracking
        self.last_target_time = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_distance = float('inf')
        self.target_angle = 0.0

        # Subscribers
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.target_sub = self.create_subscription(
            PoseStamped, '/target_pose', self._target_cb, qos_reliable)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_best_effort)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop
        period = 1.0 / self.control_rate
        self.timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"=== AUTONOMOUS EXPLORER STARTED ===\n"
            f"  Phase 1: Scanning 360° ({self.scan_duration:.1f}s) looking for target...\n"
            f"  Phase 2: Wander if no target found\n"
            f"  Speeds: wander={self.linear_speed}m/s, approach={self.approach_linear}m/s, push={self.push_speed}m/s")

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _target_cb(self, msg: PoseStamped):
        """Vision node detected a target."""
        self.last_target_time = time.time()
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_distance = math.sqrt(self.target_x**2 + self.target_y**2)
        self.target_angle = math.atan2(self.target_y, self.target_x)

    def _scan_cb(self, msg: LaserScan):
        """Process LiDAR scan for obstacle avoidance."""
        n = len(msg.ranges)
        if n == 0:
            return

        def sector_min(angle_start, angle_end):
            idx_start = int((angle_start - msg.angle_min) / msg.angle_increment)
            idx_end = int((angle_end - msg.angle_min) / msg.angle_increment)
            idx_start = max(0, min(n - 1, idx_start))
            idx_end = max(0, min(n - 1, idx_end))
            if idx_start > idx_end:
                idx_start, idx_end = idx_end, idx_start
            sector = msg.ranges[idx_start:idx_end + 1]
            valid = [r for r in sector if msg.range_min < r < msg.range_max]
            return min(valid) if valid else float('inf')

        self.min_front_dist = sector_min(-0.52, 0.52)
        self.min_left_dist = sector_min(0.52, 1.57)
        self.min_right_dist = sector_min(-1.57, -0.52)

    # -------------------------------------------------------------------------
    # State machine
    # -------------------------------------------------------------------------

    def _control_loop(self):
        now = time.time()
        target_fresh = (now - self.last_target_time) < self.target_lost_timeout

        # --- State transitions ---
        if self.state == self.SCAN_ROTATE:
            if target_fresh:
                self.get_logger().info(
                    f"TARGET FOUND during scan! dist={self.target_distance:.2f}m "
                    f"angle={math.degrees(self.target_angle):.1f}°")
                self._enter_state(self.APPROACH)
            elif now >= self.scan_end_time:
                self.get_logger().info("360° scan complete, no target. Wandering...")
                self._enter_state(self.WANDER)

        elif self.state == self.WANDER:
            if target_fresh:
                self.get_logger().info(
                    f"TARGET FOUND while wandering! dist={self.target_distance:.2f}m")
                self._enter_state(self.APPROACH)
            elif (now - self.wander_start_time) > self.wander_before_rescan:
                self.get_logger().info("Wander timeout, scanning 360° again...")
                self._enter_state(self.SCAN_ROTATE)

        elif self.state == self.AVOID_TURN:
            if target_fresh:
                self._enter_state(self.APPROACH)
            elif now >= self.turn_end_time:
                self._enter_state(self.WANDER)

        elif self.state == self.APPROACH:
            if not target_fresh:
                self.get_logger().info("Target lost, scanning again...")
                self._enter_state(self.SCAN_ROTATE)
            elif self.target_distance < self.approach_distance:
                self.get_logger().info(
                    f"Close enough ({self.target_distance:.2f}m), PUSHING!")
                self._enter_state(self.PUSH)

        elif self.state == self.PUSH:
            if now >= self.push_end_time:
                self.targets_pushed += 1
                self.get_logger().info(
                    f"=== PUSH COMPLETE! Total: {self.targets_pushed} ===")
                self._enter_state(self.BACKUP)

        elif self.state == self.BACKUP:
            if now >= self.backup_end_time:
                self.get_logger().info("Backup done, scanning for next target...")
                self._enter_state(self.SCAN_ROTATE)

        # --- Execute current state ---
        cmd = Twist()

        if self.state == self.SCAN_ROTATE:
            cmd = self._do_scan_rotate()
        elif self.state == self.WANDER:
            cmd = self._do_wander()
        elif self.state == self.AVOID_TURN:
            cmd = self._do_avoid_turn()
        elif self.state == self.APPROACH:
            cmd = self._do_approach()
        elif self.state == self.PUSH:
            cmd = self._do_push()
        elif self.state == self.BACKUP:
            cmd = self._do_backup()

        self.cmd_pub.publish(cmd)

    def _enter_state(self, new_state):
        now = time.time()
        old_state = self.state
        self.state = new_state
        self.state_start_time = now

        if new_state == self.SCAN_ROTATE:
            self.scan_end_time = now + self.scan_duration
        elif new_state == self.WANDER:
            self.wander_start_time = now
        elif new_state == self.AVOID_TURN:
            duration = random.uniform(self.turn_duration_min, self.turn_duration_max)
            self.turn_end_time = now + duration
            if self.min_left_dist < self.min_right_dist:
                self.turn_direction = -1.0
            else:
                self.turn_direction = 1.0
        elif new_state == self.PUSH:
            self.push_end_time = now + self.push_duration
        elif new_state == self.BACKUP:
            self.backup_end_time = now + self.backup_duration

        self.get_logger().info(f"State: {old_state} -> {new_state}")

    # -------------------------------------------------------------------------
    # State behaviors
    # -------------------------------------------------------------------------

    def _do_scan_rotate(self) -> Twist:
        """Rotate in place, scanning for target."""
        cmd = Twist()
        cmd.angular.z = self.scan_angular_speed
        remaining = self.scan_end_time - time.time()
        # Log progress every ~2 seconds
        elapsed = self.scan_duration - remaining
        if int(elapsed * 5) % 10 == 0 and int(elapsed * 5) > 0:
            degrees_done = math.degrees(self.scan_angular_speed * elapsed)
            self.get_logger().info(f"  Scanning... {degrees_done:.0f}°/{360}°")
        return cmd

    def _do_wander(self) -> Twist:
        """Drive forward, avoid obstacles."""
        cmd = Twist()

        if self.min_front_dist < self.obstacle_distance:
            self.get_logger().info(
                f"Obstacle at {self.min_front_dist:.2f}m, turning away")
            self._enter_state(self.AVOID_TURN)
            return Twist()

        # Slow down near obstacles but keep minimum PWM-viable speed
        if self.min_front_dist < self.obstacle_slow_distance:
            factor = (self.min_front_dist - self.obstacle_distance) / (
                self.obstacle_slow_distance - self.obstacle_distance)
            factor = max(0.5, min(1.0, factor))  # min 50% speed
            cmd.linear.x = self.linear_speed * factor
        else:
            cmd.linear.x = self.linear_speed

        # Gentle steering away from side obstacles
        if self.min_left_dist < self.obstacle_slow_distance:
            cmd.angular.z = -0.2
        elif self.min_right_dist < self.obstacle_slow_distance:
            cmd.angular.z = 0.2

        return cmd

    def _do_avoid_turn(self) -> Twist:
        """Turn in place to avoid obstacle."""
        cmd = Twist()
        cmd.angular.z = self.angular_speed * self.turn_direction
        return cmd

    def _do_approach(self) -> Twist:
        """Steer toward detected target."""
        cmd = Twist()

        if self.min_front_dist < self.obstacle_distance:
            self.get_logger().warn("Obstacle during approach, stopping")
            return cmd

        if abs(self.target_angle) > self.angle_tolerance:
            # Proportional angular control toward target
            cmd.angular.z = self.approach_angular * (
                self.target_angle / max(abs(self.target_angle), 0.3))
        else:
            # Aligned - move forward
            speed_factor = min(1.0, (self.target_distance - 0.1) / 0.5)
            speed_factor = max(0.3, speed_factor)
            cmd.linear.x = self.approach_linear * speed_factor
            # Small angular correction while moving
            cmd.angular.z = 0.3 * self.target_angle

        return cmd

    def _do_push(self) -> Twist:
        """Drive forward through the target."""
        cmd = Twist()
        cmd.linear.x = self.push_speed
        return cmd

    def _do_backup(self) -> Twist:
        """Reverse after push."""
        cmd = Twist()
        cmd.linear.x = self.backup_speed
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.get_logger().info(
            f"Shutting down. Total targets pushed: {node.targets_pushed}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
