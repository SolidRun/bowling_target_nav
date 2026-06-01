#!/usr/bin/env python3
"""
Arduino Driver Node for ROS2
=============================

ROS2 node bridging ``/cmd_vel`` velocity commands and ``/arduino/cmd``
raw firmware commands to the Arduino mecanum motor controller firmware
via USB serial. Uses the non-blocking ``ArduinoBridge`` from
``hardware/arduino_bridge.py`` with background read and reconnect threads.

Two command paths share one serial port, controlled by a ``_vel_mode`` flag:

    VEL mode (_vel_mode=True):
        Navigation publishes Twist on /cmd_vel. The 20Hz command_loop
        converts m/s to mm/s and streams VEL commands to the firmware.
        The firmware's 200ms watchdog keeps motors alive only while VEL
        commands keep arriving. On cmd_vel timeout (0.5s), STOP is sent
        once and _vel_mode flips to False.

    Position mode (_vel_mode=False):
        GUI Tools tab sends firmware commands directly via /arduino/cmd
        (e.g. FWD,100,1719 / TMOTOR,FL,100 / CALIB). The command_loop
        is idle and does not interfere. The firmware manages PID, accel
        ramp, completion (DONE), and stall detection autonomously.

Mode transitions:
    /cmd_vel received    -> _vel_mode = True  (nav takes control)
    /arduino/cmd received -> _vel_mode = False (GUI takes control)
    cmd_vel timeout      -> sends STOP once, _vel_mode = False

Topics:
    Subscribed:
        /cmd_vel (geometry_msgs/Twist)     -- Body-frame velocity commands
                                              from navigation (VEL mode).
        /arduino/cmd (std_msgs/String)     -- Raw firmware commands from GUI
                                              Tools tab (position mode).

    Published:
        /arduino/status (std_msgs/String)  -- Connection status JSON
                                              (TRANSIENT_LOCAL for late joiners).
        /arduino/odom_raw (std_msgs/String)-- ODOM telemetry JSON from firmware
                                              (consumed by OdometryNode).
        /diagnostics (DiagnosticArray)     -- TX/RX/error counts at 1 Hz.

Parameters:
    serial_port (str):        Serial port path [default: /dev/ttyACM0]
    baudrate (int):           Serial baudrate [default: 115200]
    command_rate (float):     Command send rate Hz [default: 20.0]
    command_timeout (float):  Stop motors if no cmd_vel for this long [default: 0.5s]
    enable_diagnostics (bool):Publish /diagnostics topic [default: true]

Usage:
    ros2 run target_nav arduino_driver_node
    ros2 run target_nav arduino_driver_node --ros-args -p serial_port:=/dev/ttyUSB0

Related modules:
    hardware/arduino_bridge.py -- ArduinoBridge (non-blocking serial interface).
    hardware/arduino.py        -- protocol constants, blocking model, MockArduino.
    app/odometry_node.py       -- consumes /arduino/odom_raw telemetry.
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

from target_nav.hardware.arduino import DEFAULT_ARDUINO_PORT, DEFAULT_BAUDRATE
from target_nav.hardware.arduino_bridge import ArduinoBridge, ArduinoConfig, ArduinoState


class ArduinoDriverNode(Node):
    """ROS2 node bridging velocity and position commands to the Arduino.

    Two command paths controlled by ``_vel_mode`` flag:

        VEL mode (True):  /cmd_vel -> 20Hz command_loop -> VEL,mm/s
        Position mode (False): /arduino/cmd -> direct passthrough -> FWD,TMOTOR,CALIB

    The command_loop only sends VEL/STOP when ``_vel_mode`` is True.
    When the GUI sends position commands, the loop is idle so it doesn't
    interfere with firmware-managed moves.

    Publishes connection status on ``/arduino/status`` and ODOM telemetry
    on ``/arduino/odom_raw`` (consumed by OdometryNode). Optionally
    publishes ``/diagnostics`` at 1 Hz.
    """

    def __init__(self):
        # Register this node with ROS2 under the name 'arduino_driver'.
        # Other nodes can find it via: ros2 node list -> /arduino_driver
        super().__init__('arduino_driver')

        # --- Step 1: Parameters ---
        # ROS2 parameters are like config values that can be set at launch time:
        #   ros2 run target_nav arduino_driver_node --ros-args -p serial_port:=/dev/ttyUSB0
        # We first declare them (with defaults), then read their values.
        self._declare_parameters()

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.command_rate = self.get_parameter('command_rate').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.enable_diagnostics = self.get_parameter('enable_diagnostics').value

        # --- Step 2: Internal state ---
        # Mode flag: who controls the motors right now?
        #   True  = navigation via /cmd_vel (VEL mode, command_loop active)
        #   False = GUI via /arduino/cmd (position mode, command_loop idle)
        # Navigation sets this True on each cmd_vel. GUI sets it False on
        # each arduino_cmd (FWD, TMOTOR, CALIB, etc.) so the command_loop
        # doesn't interfere with firmware-managed position commands.
        self._vel_mode = False

        # We track the last velocity command and when it arrived.
        # If no new cmd_vel arrives within command_timeout (0.5s), we send STOP
        # to the Arduino as a safety measure (the robot should not keep moving
        # if the navigation node crashes or stops publishing).
        self._last_cmd_vel_time: Optional[float] = None  # None = no cmd received yet
        self._last_cmd_vel = Twist()  # Twist has linear.x/y/z and angular.x/y/z
        self._current_state = ArduinoState.DISCONNECTED

        # --- Step 3: Arduino bridge ---
        # ArduinoBridge handles all the serial port complexity:
        #   - Opens /dev/ttyACM0 (or auto-detects the Arduino)
        #   - Runs a background thread that continuously reads serial data
        #   - Runs a background thread that auto-reconnects if USB disconnects
        #   - Provides send_velocity(), send_stop(), send_command() methods
        #
        # We give it two callbacks:
        #   on_response: called when Arduino sends data back (ODOM, ENC, ERROR)
        #   on_state_change: called when connection state changes (CONNECTED, ERROR, etc.)
        # This way the bridge does I/O, and this node handles ROS2 topics.
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

        # --- Step 4: QoS (Quality of Service) profiles ---
        # QoS controls HOW messages are delivered between ROS2 nodes.
        # Different topics need different delivery guarantees.

        # For /cmd_vel: speed matters more than reliability.
        # BEST_EFFORT = don't retry lost messages (a stale velocity is useless)
        # VOLATILE = no history kept (late subscribers start fresh)
        # depth=1 = only buffer the very latest command
        # This matches the nav stack convention for velocity commands.
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # For /arduino/status: reliability matters more than speed.
        # RELIABLE = guarantee delivery (retry if needed)
        # TRANSIENT_LOCAL = keep the last message so when a new subscriber
        #   joins (e.g. GUI starts up later), it immediately gets the current
        #   connection status without waiting for the next update.
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # --- Step 5: Callback group ---
        # ReentrantCallbackGroup allows multiple callbacks to run at the same
        # time (concurrently). Without this, ROS2 would process them one at a
        # time, which could delay the 20Hz command loop if a cmd_vel or
        # arduino/cmd callback is still executing.
        # NOTE: true parallelism requires MultiThreadedExecutor (see main()).
        # With the default single-threaded executor, callbacks still run
        # sequentially but won't block each other at the group level.
        self.callback_group = ReentrantCallbackGroup()

        # --- Step 6: Subscribers (incoming messages) ---

        # /cmd_vel: velocity commands from the navigation node.
        # Message type: Twist (linear.x = forward m/s, linear.y = strafe m/s,
        #                       angular.z = rotation rad/s)
        # When a message arrives, _cmd_vel_callback stores it for the command loop.
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            cmd_vel_qos,
            callback_group=self.callback_group
        )

        # /arduino/cmd: raw firmware commands from other nodes or the GUI.
        # Used for things like "CALIB" (calibration), "READ" (encoder read),
        # "TMOTOR" (test motor). These bypass the velocity pipeline.
        # depth=10 means buffer up to 10 pending messages.
        self.arduino_cmd_sub = self.create_subscription(
            String,
            'arduino/cmd',
            self._arduino_cmd_callback,
            10,
            callback_group=self.callback_group
        )

        # --- Step 7: Publishers (outgoing messages) ---

        # /arduino/status: publishes JSON like {"state": "connected", "port": "/dev/ttyACM0"}
        # Other nodes (GUI, health monitor) subscribe to know if Arduino is alive.
        self.status_pub = self.create_publisher(
            String,
            'arduino/status',
            status_qos
        )

        # /arduino/odom_raw: publishes raw telemetry JSON from the Arduino.
        # The OdometryNode subscribes to this and converts it into proper
        # ROS2 Odometry messages with pose and velocity.
        self.odom_raw_pub = self.create_publisher(
            String,
            'arduino/odom_raw',
            10
        )

        # /diagnostics: standard ROS2 diagnostics topic for monitoring tools
        # like rqt_robot_monitor. Shows TX/RX counts, error counts, etc.
        if self.enable_diagnostics:
            self.diag_pub = self.create_publisher(
                DiagnosticArray,
                'diagnostics',
                10
            )

        # --- Step 8: Timers (periodic callbacks) ---

        # Command loop timer: fires at command_rate Hz (default 20Hz = every 50ms).
        # Each tick, it takes the latest cmd_vel and sends it to the Arduino
        # as a VEL command. This keeps the firmware watchdog happy (200ms timeout).
        # If no cmd_vel was received recently, it sends STOP instead.
        # 20Hz was chosen because: firmware watchdog = 200ms, so we need >= 5Hz,
        # and 20Hz gives 4x margin for jitter/delays.
        self.command_timer = self.create_timer(
            1.0 / self.command_rate,  # period in seconds (1/20 = 0.05s = 50ms)
            self._command_loop,
            callback_group=self.callback_group
        )

        # Diagnostics timer: fires at 1Hz to publish health/stats info.
        if self.enable_diagnostics:
            self.diag_timer = self.create_timer(
                1.0,  # 1 Hz = once per second
                self._publish_diagnostics,
                callback_group=self.callback_group
            )

        # --- Step 9: Start ---
        # bridge.start() launches two background daemon threads:
        #   1. _read_loop: reads serial lines and calls on_response callback
        #   2. _reconnect_loop: monitors connection and auto-reconnects
        # From this point on, the bridge is actively communicating with Arduino.
        self.bridge.start()

        self.get_logger().info(
            f'Arduino driver started - configured port: {self.serial_port} '
            f'(auto-detect may select another), rate: {self.command_rate}Hz'
        )

    def _report_port(self) -> str:
        """Return the device the bridge actually opened, for logs/status.

        Auto-detection may select a device other than the configured
        ``serial_port`` (e.g. when the default doesn't exist). Prefer the
        bridge's real port; fall back to the configured value before the
        first connection completes.
        """
        return self.bridge.actual_port or self.serial_port

    def _declare_parameters(self):
        """Declare all ROS2 parameters with their default values."""
        self.declare_parameter('serial_port', DEFAULT_ARDUINO_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('command_rate', 20.0)        # Hz (must be ≥5 for 200ms watchdog)
        self.declare_parameter('command_timeout', 0.5)      # seconds
        self.declare_parameter('enable_diagnostics', True)

    def _cmd_vel_callback(self, msg: Twist):
        """Called every time a new velocity command arrives on /cmd_vel.

        We don't send it to Arduino here -- we just store it. The command_loop
        timer (20Hz) picks up the latest stored value and sends it. This
        decouples the nav publish rate from the Arduino send rate.

        Sets _vel_mode = True so the command_loop takes over motor control.
        This preempts any GUI position command (FWD, TMOTOR) that was running.
        """
        self._vel_mode = True
        self._last_cmd_vel = msg
        self._last_cmd_vel_time = time.time()  # Reset the timeout clock
        self.get_logger().debug(f'cmd_vel: x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, wz={msg.angular.z:.2f}')

    def _arduino_cmd_callback(self, msg: String):
        """Called when a raw command arrives on /arduino/cmd topic.

        This is a "passthrough" for non-velocity commands like:
          ros2 topic pub /arduino/cmd std_msgs/String "data: CALIB"
          ros2 topic pub /arduino/cmd std_msgs/String "data: READ"
        The GUI also uses this to send calibration/test commands.
        These go directly to the firmware, bypassing the velocity pipeline.
        """
        command = msg.data.strip().upper()
        self.get_logger().info(f'Received Arduino command: {command}')

        # Switch to position mode: disable the VEL command_loop so it
        # doesn't send STOP and interfere with firmware position commands
        # (FWD, TURN, TMOTOR, CALIB, etc.). Navigation resumes VEL mode
        # when the next cmd_vel arrives.
        # Also reset _last_cmd_vel_time so the command_loop has no stale
        # timestamp to act on (prevents a race where one more STOP slips through).
        self._vel_mode = False
        self._last_cmd_vel_time = None

        result = self.bridge.send_command(command)
        if result:
            self.get_logger().info(f'Arduino command sent: {command}')
        else:
            self.get_logger().warning(f'Failed to send Arduino command: {command}')

    def _handle_arduino_response(self, command: str, args: list):
        """Called by ArduinoBridge's background read thread when a line arrives
        from the Arduino serial port.

        The bridge parses each line by splitting on commas:
          "ODOM,120,-30,5" -> command="ODOM", args=["120","-30","5"]
          "ERROR:something" -> command="ERROR", args=["something"]

        We convert ODOM into JSON and publish on /arduino/odom_raw so the
        OdometryNode can consume it without knowing about serial at all.
        ENC telemetry (raw encoder ticks sent during IDLE/TESTING) is ignored
        — the firmware already computes forward kinematics in VEL mode and
        sends ODOM directly, which is the only mode used during navigation.
        """
        if command == "ODOM":
            # ODOM telemetry: firmware computed velocities from encoder deltas.
            # args = ["vx_mm", "vy_mm", "wz_mrad"] as strings.
            # We convert to floats and publish as JSON for OdometryNode.
            msg = String()
            msg.data = json.dumps({
                'command': 'ODOM',
                'args': [float(a) for a in args if a.lstrip('-').isdigit()],
                'timestamp': time.time()
            })
            self.odom_raw_pub.publish(msg)

        elif command == "ERROR":
            self.get_logger().warning(f"Arduino error: {args}")

    def _handle_state_change(self, new_state: ArduinoState):
        """Called by ArduinoBridge when the connection state changes.

        State machine: DISCONNECTED -> CONNECTING -> CONNECTED -> ERROR -> ...
        The bridge's reconnect thread automatically cycles through these.
        We publish the state as JSON on /arduino/status (TRANSIENT_LOCAL so
        the GUI gets the current state immediately when it subscribes).
        """
        self._current_state = new_state

        # Publish as JSON so any subscriber can parse it
        port = self._report_port()
        status_msg = String()
        status_msg.data = json.dumps({
            'state': new_state.value,       # e.g. "connected", "error"
            'port': port,                   # actually-opened device (auto-detected)
            'timestamp': time.time()
        })
        self.status_pub.publish(status_msg)

        if new_state == ArduinoState.CONNECTED:
            self.get_logger().info(f"Connected to Arduino on {port}")
        elif new_state == ArduinoState.ERROR:
            self.get_logger().warning("Arduino connection error - will retry")
        elif new_state == ArduinoState.DISCONNECTED:
            self.get_logger().info("Arduino disconnected")

    def _command_loop(self):
        """THE CORE LOOP -- fires at 20Hz (every 50ms).

        This is the heartbeat of the motor control. Every tick it does:
          1. Check if we ever received a cmd_vel. If not, do nothing (idle).
          2. Check if cmd_vel is stale (older than 0.5s). If so, send STOP.
             This is a safety feature: if the nav node crashes, motors stop.
          3. Otherwise, take the latest cmd_vel, convert units, and send VEL.

        Why 20Hz? The firmware has a 200ms watchdog. If it doesn't receive
        a VEL command within 200ms, it auto-stops the motors. At 20Hz we
        send every 50ms, giving 4x safety margin for timing jitter.

        Why convert units? ROS2 uses SI units (m/s, rad/s), but the firmware
        uses integer mm/s and mrad/s to avoid floating point on the Arduino.
        Example: linear.x = 0.15 m/s -> 150 mm/s, angular.z = 0.5 rad/s -> 500 mrad/s
        """
        # Only send VEL/STOP when navigation is in control.
        # When GUI sends position commands (FWD, TMOTOR, etc.), _vel_mode
        # is False and we stay out of the way — the firmware handles it.
        if not self._vel_mode:
            return

        current_time = time.time()

        # No cmd_vel received yet -> robot is idle, don't send anything
        if self._last_cmd_vel_time is None:
            return

        time_since_cmd = current_time - self._last_cmd_vel_time

        # Safety timeout: if navigation stopped publishing, stop the motors.
        # This prevents the robot from driving forever if the nav node dies.
        if time_since_cmd > self.command_timeout:
            self.bridge.send_stop()
            self._vel_mode = False  # Stop once, then go idle
            return

        # Unit conversion: ROS2 m/s -> firmware mm/s, ROS2 rad/s -> firmware mrad/s
        vx_mm = int(self._last_cmd_vel.linear.x * 1000)    # forward/backward
        vy_mm = int(self._last_cmd_vel.linear.y * 1000)    # left/right strafe (mecanum)
        wz_mrad = int(self._last_cmd_vel.angular.z * 1000) # rotation

        # Send VEL command to Arduino via serial bridge.
        # The bridge also applies a deadzone: if all values < 3, it sends STOP instead.
        result = self.bridge.send_velocity(vx_mm, vy_mm, wz_mrad)
        if not result:
            self.get_logger().warning('Failed to send velocity command to Arduino')

    def _publish_diagnostics(self):
        """Fires at 1Hz -- publishes health info on the standard /diagnostics topic.

        This is a ROS2 convention: tools like rqt_robot_monitor subscribe to
        /diagnostics to show a dashboard of all hardware components and their
        health. We report:
          - OK (green) if connected
          - WARN (yellow) if connecting
          - ERROR (red) if disconnected or error
          - TX/RX/error/reconnect counts for debugging communication issues
        """
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "Arduino Driver"
        status.hardware_id = self._report_port()

        # Map our connection state to the standard diagnostic levels
        if self._current_state == ArduinoState.CONNECTED:
            status.level = DiagnosticStatus.OK
            status.message = "Connected and operational"
        elif self._current_state == ArduinoState.CONNECTING:
            status.level = DiagnosticStatus.WARN
            status.message = "Attempting to connect"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = f"State: {self._current_state.value}"

        # Stats from the bridge -- useful for debugging:
        # tx_count: how many commands we sent to Arduino
        # rx_count: how many lines we received from Arduino
        # errors: serial communication errors (timeouts, broken pipe, etc.)
        # reconnects: how many times the bridge had to reconnect
        stats = self.bridge.stats
        status.values = [
            KeyValue(key="state", value=self._current_state.value),
            KeyValue(key="port", value=self._report_port()),
            KeyValue(key="tx_count", value=str(stats['tx_count'])),
            KeyValue(key="rx_count", value=str(stats['rx_count'])),
            KeyValue(key="errors", value=str(stats['errors'])),
            KeyValue(key="reconnects", value=str(stats['reconnects'])),
        ]

        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)

    def destroy_node(self):
        """Clean shutdown -- called when the node is killed (Ctrl+C, ros2 lifecycle, etc.).

        Order matters:
          1. Send STOP so motors don't keep running after we disconnect
          2. Stop the bridge's background threads (read + reconnect)
          3. Call parent destroy_node() to clean up ROS2 resources
        """
        self.get_logger().info("Shutting down Arduino driver...")
        self.bridge.send_stop()
        self.bridge.stop()
        super().destroy_node()


def main(args=None):
    """Entry point -- wired up in setup.py as 'arduino_driver_node' console script.

    The lifecycle:
      1. rclpy.init() -- initialize the ROS2 client library
      2. Create the node -- sets up all subscribers, publishers, timers, bridge
      3. rclpy.spin() -- blocks here, processing callbacks forever until interrupted
      4. On Ctrl+C (KeyboardInterrupt) or shutdown signal:
         - destroy_node() sends STOP and cleans up
         - rclpy.shutdown() tears down the ROS2 context
    """
    rclpy.init(args=args)

    node = ArduinoDriverNode()

    try:
        # spin() blocks and processes all callbacks (timers, subscribers)
        # in a single-threaded executor. The node stays alive here until killed.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
