#!/usr/bin/env python3
"""
Odometry Node for ROS2
======================

Converts raw Arduino odometry data to proper nav_msgs/Odometry
and publishes TF transform (odom -> base_link).

Subscribed Topics:
    /arduino/odom_raw (std_msgs/String) - Raw odometry JSON from Arduino

Published Topics:
    /odom (nav_msgs/Odometry) - Standard odometry message

TF Broadcasts:
    odom -> base_link

Parameters:
    odom_frame: Frame ID for odometry (default: "odom")
    base_frame: Robot base frame (default: "base_link")
    publish_tf: Whether to broadcast TF (default: true)
    wheel_separation: Distance between wheels in meters (default: 0.3)
    wheel_radius: Wheel radius in meters (default: 0.05)
"""

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle to quaternion (roll=0, pitch=0)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdometryNode(Node):
    """Converts Arduino raw odometry to nav_msgs/Odometry + TF."""

    def __init__(self):
        super().__init__('odometry_node')

        # Declare parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('wheel_separation', 0.3)  # meters
        self.declare_parameter('wheel_radius', 0.05)     # meters

        # Get parameters
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_time: Optional[float] = None

        # QoS
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        # Subscriber for raw Arduino odometry
        self.raw_odom_sub = self.create_subscription(
            String,
            'arduino/odom_raw',
            self._raw_odom_callback,
            10
        )

        # Publisher for standard odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            odom_qos
        )

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'Odometry node started: {self.odom_frame} -> {self.base_frame}'
        )

    def _raw_odom_callback(self, msg: String):
        """Process raw odometry from Arduino."""
        try:
            data = json.loads(msg.data)

            # Expected format from arduino_driver_node:
            # {"command": "ODOM", "args": [left_ticks, right_ticks, ...], "timestamp": ...}
            if data.get('command') != 'ODOM':
                return

            args = data.get('args', [])
            timestamp = data.get('timestamp', 0.0)

            # Parse odometry data based on Arduino protocol
            # Adjust this based on your actual Arduino output format
            if len(args) >= 2:
                # Simple format: [linear_vel_mm, angular_vel_mrad]
                linear_vel_mm = float(args[0])
                angular_vel_mrad = float(args[1])

                # Convert to m/s and rad/s
                self.vx = linear_vel_mm / 1000.0
                self.vth = angular_vel_mrad / 1000.0

            elif len(args) >= 4:
                # Extended format: [x_mm, y_mm, theta_mrad, ...]
                self.x = float(args[0]) / 1000.0
                self.y = float(args[1]) / 1000.0
                self.theta = float(args[2]) / 1000.0

            # Integrate if we have velocity data
            current_time = timestamp
            if self.last_time is not None:
                dt = current_time - self.last_time
                if 0.0 < dt < 1.0:  # Sanity check
                    # Update position
                    self.x += self.vx * math.cos(self.theta) * dt
                    self.y += self.vx * math.sin(self.theta) * dt
                    self.theta += self.vth * dt

                    # Normalize theta to [-pi, pi]
                    while self.theta > math.pi:
                        self.theta -= 2.0 * math.pi
                    while self.theta < -math.pi:
                        self.theta += 2.0 * math.pi

            self.last_time = current_time

            # Publish odometry
            self._publish_odometry()

        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON in odom_raw: {msg.data[:50]}")
        except Exception as e:
            self.get_logger().error(f"Odometry processing error: {e}")

    def _publish_odometry(self):
        """Publish odometry message and TF."""
        now = self.get_clock().now()

        # Create quaternion from yaw
        q = euler_to_quaternion(self.theta)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        # Covariance (simple diagonal)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.01  # yaw
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[35] = 0.01 # vth

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
