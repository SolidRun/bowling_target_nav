#!/usr/bin/env python3
"""
Professional Arduino Driver Node for ROS2

This node follows production ROS2 patterns:
- ros2_control-style hardware interface
- Diagnostic publishing for monitoring
- Lifecycle node pattern (simplified)
- Parameter-driven configuration
- Proper QoS settings

Topics:
    Subscribed:
        /cmd_vel (geometry_msgs/Twist) - Velocity commands

    Published:
        /arduino/status (std_msgs/String) - Connection status JSON
        /diagnostics (diagnostic_msgs/DiagnosticArray) - System diagnostics
        /arduino/odom_raw (std_msgs/String) - Raw odometry from Arduino

Parameters:
    serial_port (str): Serial port path [default: /dev/ttyACM0]
    baudrate (int): Serial baudrate [default: 115200]
    linear_scale (float): m/s to mm/s conversion [default: 1000.0]
    angular_scale (float): rad/s to mrad/s conversion [default: 1000.0]
    max_linear_vel (float): Max linear velocity mm/s [default: 200.0]
    max_angular_vel (float): Max angular velocity mrad/s [default: 200.0]
    command_rate (float): Command publish rate Hz [default: 20.0]
    command_timeout (float): Stop if no cmd_vel for this long [default: 0.5]
    enable_diagnostics (bool): Publish diagnostics [default: true]

Usage:
    ros2 run bowling_target_nav arduino_driver_node
    ros2 run bowling_target_nav arduino_driver_node --ros-args -p serial_port:=/dev/ttyUSB0
"""

import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from bowling_target_nav.hardware import ArduinoBridge, ArduinoConfig, ArduinoState


class ArduinoDriverNode(Node):
    """
    Professional ROS2 Arduino driver following best practices.
    """

    def __init__(self):
        super().__init__('arduino_driver')

        # Declare parameters with defaults
        self._declare_parameters()

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.command_rate = self.get_parameter('command_rate').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.enable_diagnostics = self.get_parameter('enable_diagnostics').value

        # State tracking
        self._last_cmd_vel_time: Optional[float] = None
        self._last_cmd_vel = Twist()
        self._current_state = ArduinoState.DISCONNECTED

        # Initialize Arduino bridge
        config = ArduinoConfig(
            port=self.serial_port,
            baudrate=self.baudrate,
            command_timeout=self.command_timeout
        )

        self.bridge = ArduinoBridge(
            config=config,
            on_response=self._handle_arduino_response,
            on_state_change=self._handle_state_change
        )

        # Setup QoS profiles
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            cmd_vel_qos,
            callback_group=self.callback_group
        )

        # Direct Arduino command subscriber (for calibration, read, etc.)
        self.arduino_cmd_sub = self.create_subscription(
            String,
            'arduino/cmd',
            self._arduino_cmd_callback,
            10,
            callback_group=self.callback_group
        )

        # Publishers
        self.status_pub = self.create_publisher(
            String,
            'arduino/status',
            status_qos
        )

        self.odom_raw_pub = self.create_publisher(
            String,
            'arduino/odom_raw',
            10
        )

        if self.enable_diagnostics:
            self.diag_pub = self.create_publisher(
                DiagnosticArray,
                'diagnostics',
                10
            )

        # Timers
        self.command_timer = self.create_timer(
            1.0 / self.command_rate,
            self._command_loop,
            callback_group=self.callback_group
        )

        if self.enable_diagnostics:
            self.diag_timer = self.create_timer(
                1.0,  # 1 Hz diagnostics
                self._publish_diagnostics,
                callback_group=self.callback_group
            )

        # Start the bridge
        self.bridge.start()

        self.get_logger().info(
            f'Arduino driver started - port: {self.serial_port}, '
            f'rate: {self.command_rate}Hz'
        )

    def _declare_parameters(self):
        """Declare all ROS2 parameters."""
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('linear_scale', 1000.0)   # m/s -> mm/s
        self.declare_parameter('angular_scale', 1000.0)  # rad/s -> mrad/s
        self.declare_parameter('max_linear_vel', 200.0)  # mm/s
        self.declare_parameter('max_angular_vel', 200.0) # mrad/s
        self.declare_parameter('command_rate', 20.0)     # Hz
        self.declare_parameter('command_timeout', 0.5)   # seconds
        self.declare_parameter('enable_diagnostics', True)

    def _cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands."""
        self._last_cmd_vel = msg
        self._last_cmd_vel_time = time.time()
        self.get_logger().info(f'Received cmd_vel: x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, z={msg.angular.z:.2f}')

    def _arduino_cmd_callback(self, msg: String):
        """Handle direct Arduino commands (SYNC, READ, RESET, etc.)."""
        command = msg.data.strip().upper()
        self.get_logger().info(f'Received Arduino command: {command}')

        # Send command directly to Arduino
        result = self.bridge.send_command(command)
        if result:
            self.get_logger().info(f'Arduino command sent: {command}')
        else:
            self.get_logger().warn(f'Failed to send Arduino command: {command}')

    def _handle_arduino_response(self, command: str, args: list):
        """Handle responses from Arduino."""
        if command == "ODOM":
            # Forward raw odometry data
            msg = String()
            msg.data = json.dumps({
                'command': command,
                'args': args,
                'timestamp': time.time()
            })
            self.odom_raw_pub.publish(msg)

        elif command == "STATUS":
            self.get_logger().debug(f"Arduino status: {args}")

        elif command == "ERROR":
            self.get_logger().warn(f"Arduino error: {args}")

    def _handle_state_change(self, new_state: ArduinoState):
        """Handle connection state changes."""
        self._current_state = new_state

        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            'state': new_state.value,
            'port': self.serial_port,
            'timestamp': time.time()
        })
        self.status_pub.publish(status_msg)

        # Log state changes
        if new_state == ArduinoState.CONNECTED:
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        elif new_state == ArduinoState.ERROR:
            self.get_logger().warn("Arduino connection error - will retry")
        elif new_state == ArduinoState.DISCONNECTED:
            self.get_logger().info("Arduino disconnected")

    def _command_loop(self):
        """Main control loop - runs at command_rate Hz."""
        current_time = time.time()

        # Check for command timeout
        if self._last_cmd_vel_time is None:
            # No command received yet
            return

        time_since_cmd = current_time - self._last_cmd_vel_time

        if time_since_cmd > self.command_timeout:
            # Timeout - send stop command
            self.bridge.send_stop()
            return

        # Convert and clamp velocities (mecanum: x=forward, y=strafe, z=rotation)
        vx_mm = int(self._last_cmd_vel.linear.x * self.linear_scale)
        vy_mm = int(self._last_cmd_vel.linear.y * self.linear_scale)
        wz_mrad = int(self._last_cmd_vel.angular.z * self.angular_scale)

        # Apply limits
        vx_mm = max(-self.max_linear_vel, min(self.max_linear_vel, vx_mm))
        vy_mm = max(-self.max_linear_vel, min(self.max_linear_vel, vy_mm))
        wz_mrad = max(-self.max_angular_vel, min(self.max_angular_vel, wz_mrad))

        # Send to Arduino (full mecanum velocity)
        self.get_logger().info(f'Sending to Arduino: vx={vx_mm}, vy={vy_mm}, wz={wz_mrad}')
        result = self.bridge.send_velocity(int(vx_mm), int(vy_mm), int(wz_mrad))
        if not result:
            self.get_logger().warn('Failed to send velocity command to Arduino')

    def _publish_diagnostics(self):
        """Publish diagnostic information."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "Arduino Driver"
        status.hardware_id = self.serial_port

        # Determine status level
        if self._current_state == ArduinoState.CONNECTED:
            status.level = DiagnosticStatus.OK
            status.message = "Connected and operational"
        elif self._current_state == ArduinoState.CONNECTING:
            status.level = DiagnosticStatus.WARN
            status.message = "Attempting to connect"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = f"State: {self._current_state.value}"

        # Add key-value pairs
        stats = self.bridge.stats
        status.values = [
            KeyValue(key="state", value=self._current_state.value),
            KeyValue(key="port", value=self.serial_port),
            KeyValue(key="tx_count", value=str(stats['tx_count'])),
            KeyValue(key="rx_count", value=str(stats['rx_count'])),
            KeyValue(key="checksum_errors", value=str(stats['checksum_errors'])),
            KeyValue(key="reconnects", value=str(stats['reconnects'])),
        ]

        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)

    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info("Shutting down Arduino driver...")
        self.bridge.send_stop()
        self.bridge.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ArduinoDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
