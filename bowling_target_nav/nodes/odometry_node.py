#!/usr/bin/env python3
"""
Odometry Node for ROS2
======================

Converts raw Arduino odometry data to proper nav_msgs/Odometry
and publishes TF transform (odom -> base_link).

The firmware sends ODOM,vx_mm,vy_mm,wz_mrad at 20Hz during VEL mode,
where vx/vy are in mm/s and wz is in millirad/s (from forward kinematics).

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

        # Get parameters
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Odometry state (3-DOF mecanum: x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0   # m/s forward
        self.vy = 0.0   # m/s lateral (mecanum)
        self.vth = 0.0   # rad/s yaw
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

        # Publish at fixed rate (20Hz) even when no new data arrives
        # This ensures Cartographer and Nav2 always have fresh odom + TF
        self.publish_timer = self.create_timer(0.05, self._publish_odometry)

        self.get_logger().info(
            f'Odometry node started: {self.odom_frame} -> {self.base_frame}'
        )

    def _raw_odom_callback(self, msg: String):
        """Process raw odometry from Arduino.

        Firmware sends ODOM,vx_mm,vy_mm,wz_mrad via forward kinematics.
        Driver publishes: {"command": "ODOM", "args": [vx_mm, vy_mm, wz_mrad], "timestamp": ...}
        """
        try:
            data = json.loads(msg.data)

            if data.get('command') != 'ODOM':
                return

            args = data.get('args', [])
            timestamp = data.get('timestamp', 0.0)

            if len(args) < 3:
                return

            # Firmware units: vx/vy in mm/s, wz in millirad/s
            self.vx = float(args[0]) / 1000.0   # mm/s -> m/s
            self.vy = float(args[1]) / 1000.0   # mm/s -> m/s
            self.vth = float(args[2]) / 1000.0  # mrad/s -> rad/s

            # Integrate velocity to get position
            current_time = timestamp
            if self.last_time is not None:
                dt = current_time - self.last_time
                if 0.0 < dt < 1.0:  # Sanity check
                    # Rotate body-frame velocities to world frame
                    cos_th = math.cos(self.theta)
                    sin_th = math.sin(self.theta)
                    self.x += (self.vx * cos_th - self.vy * sin_th) * dt
                    self.y += (self.vx * sin_th + self.vy * cos_th) * dt
                    self.theta += self.vth * dt

                    # Normalize theta to [-pi, pi]
                    self.theta = math.atan2(
                        math.sin(self.theta), math.cos(self.theta)
                    )

            self.last_time = current_time

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

        # Velocity (body frame)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        # Covariance (simple diagonal)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.01  # yaw
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[7] = 0.01  # vy
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
