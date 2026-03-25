#!/usr/bin/env python3
"""
Odometry Node for ROS2
======================

Converts Arduino encoder telemetry into standard nav_msgs/Odometry and
broadcasts the ``odom -> base_link`` TF transform for localization
and navigation.

The Arduino firmware sends two telemetry formats via ArduinoDriverNode:

    ODOM,vx_mm,vy_mm,wz_mrad  -- Body-frame velocities from firmware
        forward kinematics (encoder deltas). Sent at 20 Hz during VEL
        mode. Units: mm/s and millirad/s.

    ENC,FL:ticks,FR:ticks,RL:ticks,RR:ticks,t_us:us  -- Raw encoder
        positions during position mode and idle (1-20 Hz). This node
        computes wheel distances via mecanum FK and derives velocities.

Covariance strategy: Pose covariance grows linearly with accumulated
distance traveled (``dist_factor = 1 + distance * 0.1``) to reflect
mecanum wheel slip accumulation. Lateral (Y) covariance is 2x forward
(X) because mecanum rollers slip more during strafing. Unused DOFs
(z, roll, pitch) are set to 1e6 to signal downstream consumers that
those dimensions are unobserved.

Subscribed Topics:
    /arduino/odom_raw (std_msgs/String) -- JSON telemetry from driver node.
    /reset_odom (std_msgs/Empty)        -- Reset pose to origin.

Published Topics:
    /odom (nav_msgs/Odometry) -- Standard odometry with covariance.

TF Broadcasts:
    odom -> base_link (at 20 Hz, even without new data).

Related modules:
    app/arduino_node.py   -- publishes /arduino/odom_raw consumed here.
    hardware/arduino.py   -- WHEEL_RADIUS_M, WHEELBASE_M, TRACK_WIDTH_M,
                             ENCODER_CPR constants used for kinematics.
"""

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

from target_nav.hardware.arduino import (
    WHEEL_RADIUS_M, WHEELBASE_M, TRACK_WIDTH_M, ENCODER_CPR,
)


def euler_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle to quaternion (roll=0, pitch=0)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdometryNode(Node):
    """ROS2 node that converts Arduino telemetry to nav_msgs/Odometry and TF.

    Subscribes to ``/arduino/odom_raw`` (JSON strings from ArduinoDriverNode)
    and handles two telemetry formats:

        ODOM: Body-frame velocities (mm/s, mrad/s) from firmware forward
            kinematics. Directly integrated into world-frame pose.
        ENC: Raw encoder tick positions. Node computes wheel distances
            via mecanum forward kinematics and derives velocities.

    Publishes ``/odom`` (nav_msgs/Odometry) with proper covariance
    matrices. Pose covariance grows with distance traveled to reflect
    mecanum wheel slip accumulation. Unused DOFs (z, roll, pitch) have
    covariance set to 1e6 to signal that those dimensions are
    unobserved.

    Broadcasts ``odom -> base_link`` TF at 20 Hz (even without new data)
    to keep TF consumers (NavNode, GUI) supplied with fresh transforms.

    Subscribes to ``/reset_odom`` (std_msgs/Empty) to reset pose to origin.

    Parameters:
        odom_frame (str): Parent frame ID [default: 'odom'].
        base_frame (str): Child frame ID [default: 'base_link'].
        publish_tf (bool): Whether to broadcast TF [default: True].
    """

    # Covariance values for mecanum drive (diagonal entries)
    # Mecanum wheels have significant lateral slip, so vy covariance is higher
    POSE_COV_X = 0.005       # Position x (accumulated, grows with distance)
    POSE_COV_Y = 0.010       # Position y (mecanum lateral slip)
    POSE_COV_YAW = 0.01      # Heading (gyro-less, drift accumulates)
    TWIST_COV_VX = 0.01      # Linear x velocity
    TWIST_COV_VY = 0.02      # Linear y velocity (noisier due to mecanum)
    TWIST_COV_VYAW = 0.02    # Angular velocity

    # Very high covariance for unused DOFs (z, roll, pitch)
    # Signals downstream consumers these dimensions are unobserved
    UNUSED_COV = 1e6

    def __init__(self):
        super().__init__('odometry_node')

        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        # Note: use_sim_time is auto-declared by ROS2 Node base class

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Odometry state (3-DOF mecanum: x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.last_time: Optional[float] = None

        # Encoder-based odometry state (for ENC telemetry)
        self._last_enc = None  # {FL: ticks, RL: ticks, RR: ticks, FR: ticks}
        self._last_enc_time_us = 0

        # Accumulated distance for growing pose covariance
        self._distance_traveled = 0.0

        # QoS
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        # Subscriber
        self.raw_odom_sub = self.create_subscription(
            String, 'arduino/odom_raw', self._raw_odom_callback, 10
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', odom_qos)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Reset odometry subscriber
        self.create_subscription(Empty, 'reset_odom', self._reset_odom_callback, 1)

        # Publish at 20Hz even when no new data (TF consumers need fresh transforms)
        self.publish_timer = self.create_timer(0.05, self._publish_odometry)

        self.get_logger().info(
            f'Odometry node started: {self.odom_frame} -> {self.base_frame}'
        )

    def _reset_odom_callback(self, msg):
        """Reset odometry to origin."""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self._distance_traveled = 0.0
        self._last_enc = None
        self.last_time = None
        self.get_logger().info('Odometry reset to origin')

    def _raw_odom_callback(self, msg: String):
        """Process telemetry from Arduino driver node.

        Handles two formats:
        1. ODOM: {"command": "ODOM", "args": [vx_mm, vy_mm, wz_mrad], "timestamp": ...}
        2. ENC:  {"command": "ENC", "args": {"FL": t, "FR": t, "RL": t, "RR": t, "t_us": t}, "timestamp": ...}
        """
        try:
            data = json.loads(msg.data)
            command = data.get('command')
            timestamp = data.get('timestamp', 0.0)

            if command == 'ODOM':
                self._process_odom(data.get('args', []), timestamp)
            elif command == 'ENC':
                self._process_enc(data.get('args', {}), timestamp)

        except json.JSONDecodeError:
            self.get_logger().warning(f"Invalid JSON in odom_raw: {msg.data[:60]}", throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f"Odometry error: {e}", throttle_duration_sec=5.0)

    def _process_odom(self, args: list, timestamp: float):
        """Process ODOM telemetry (firmware forward kinematics).

        Firmware sends body-frame velocities from encoder deltas.
        Units: vx/vy in mm/s, wz in millirad/s.
        """
        if len(args) < 3:
            return

        self.vx = float(args[0]) / 1000.0   # mm/s -> m/s
        self.vy = float(args[1]) / 1000.0
        self.vth = float(args[2]) / 1000.0  # mrad/s -> rad/s

        self._integrate(timestamp)

    def _process_enc(self, args: dict, timestamp: float):
        """Process ENC telemetry (raw encoder positions).

        Computes velocities from encoder deltas using mecanum forward kinematics.
        Encoder order: FL(0), RL(1), RR(2), FR(3).
        """
        if not isinstance(args, dict):
            return

        enc = {}
        for key in ('FL', 'RL', 'RR', 'FR'):
            if key in args:
                enc[key] = int(args[key])
        t_us = args.get('t_us', 0)

        if len(enc) < 4:
            return

        if self._last_enc is not None and self._last_enc_time_us > 0:
            dt_us = t_us - self._last_enc_time_us
            if dt_us <= 0:
                dt_us = 20000  # Default 20ms if timestamp not advancing

            dt = dt_us / 1e6  # microseconds to seconds

            # Compute encoder deltas (ticks since last reading)
            d_fl = enc['FL'] - self._last_enc['FL']
            d_fr = enc['FR'] - self._last_enc['FR']
            d_rl = enc['RL'] - self._last_enc['RL']
            d_rr = enc['RR'] - self._last_enc['RR']

            # Convert encoder ticks to wheel travel distance (meters).
            # Each full wheel revolution = ENCODER_CPR ticks = 2*pi*R meters.
            meters_per_tick = (math.pi * 2.0 * WHEEL_RADIUS_M) / ENCODER_CPR
            dist_fl = d_fl * meters_per_tick
            dist_fr = d_fr * meters_per_tick
            dist_rl = d_rl * meters_per_tick
            dist_rr = d_rr * meters_per_tick

            # Mecanum forward kinematics: derive body-frame velocities from
            # individual wheel distances. Standard equations for X-config
            # mecanum with roller axes at 45 degrees:
            #   vx  = (FL + FR + RL + RR) / 4
            #   vy  = (-FL + FR + RL - RR) / 4
            #   wz  = (-FL + FR - RL + RR) / (4 * (L+W)/2)
            lw = (WHEELBASE_M + TRACK_WIDTH_M) / 2.0
            self.vx = (dist_fl + dist_fr + dist_rl + dist_rr) / (4.0 * dt)
            self.vy = (-dist_fl + dist_fr + dist_rl - dist_rr) / (4.0 * dt)
            self.vth = (-dist_fl + dist_fr - dist_rl + dist_rr) / (4.0 * lw * dt)

            self._integrate(timestamp)

        self._last_enc = enc
        self._last_enc_time_us = t_us

    def _integrate(self, timestamp: float):
        """Integrate body-frame velocities into world-frame pose (x, y, theta).

        Uses simple Euler integration with a delta-time guard (skips if
        dt <= 0 or dt >= 1s to reject stale or wrapped timestamps).
        Normalizes theta to [-pi, pi] after each step.

        Args:
            timestamp: Current telemetry timestamp (epoch seconds).
        """
        if self.last_time is not None:
            dt = timestamp - self.last_time
            if 0.0 < dt < 1.0:
                # Rotate body-frame velocities to world frame
                cos_th = math.cos(self.theta)
                sin_th = math.sin(self.theta)
                dx = (self.vx * cos_th - self.vy * sin_th) * dt
                dy = (self.vx * sin_th + self.vy * cos_th) * dt
                self.x += dx
                self.y += dy
                self.theta += self.vth * dt

                # Normalize theta to [-pi, pi]
                self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

                # Track distance for growing covariance
                self._distance_traveled += math.sqrt(dx * dx + dy * dy)

        self.last_time = timestamp

    def _publish_odometry(self):
        """Publish Odometry message and broadcast odom->base_link TF.

        Called at 20 Hz by a timer. Covariance matrices are populated
        with diagonal values that grow with accumulated distance to
        reflect mecanum drift. Unused DOFs are set to 1e6.
        """
        now = self.get_clock().now()
        q = euler_to_quaternion(self.theta)

        # TF
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

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        # Pose covariance (6x6 row-major: x, y, z, roll, pitch, yaw)
        # Grows with distance traveled (mecanum drift)
        dist_factor = 1.0 + self._distance_traveled * 0.1
        pc = odom.pose.covariance
        pc[0] = self.POSE_COV_X * dist_factor    # x-x
        pc[7] = self.POSE_COV_Y * dist_factor    # y-y
        pc[14] = self.UNUSED_COV                  # z-z (unused)
        pc[21] = self.UNUSED_COV                  # roll-roll (unused)
        pc[28] = self.UNUSED_COV                  # pitch-pitch (unused)
        pc[35] = self.POSE_COV_YAW * dist_factor  # yaw-yaw

        # Velocity (body frame)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        # Twist covariance
        tc = odom.twist.covariance
        tc[0] = self.TWIST_COV_VX      # vx-vx
        tc[7] = self.TWIST_COV_VY      # vy-vy
        tc[14] = self.UNUSED_COV       # vz-vz (unused)
        tc[21] = self.UNUSED_COV       # roll_rate (unused)
        tc[28] = self.UNUSED_COV       # pitch_rate (unused)
        tc[35] = self.TWIST_COV_VYAW   # yaw_rate

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
