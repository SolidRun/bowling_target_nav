#!/usr/bin/env python3
"""
Arduino Motor Controller Test Suite
====================================

Comprehensive tests for Arduino motor controller communication.
Includes connection tests, command tests, and interactive GUI for manual testing.

Tests:
    - Connection and initialization
    - Serial communication
    - Motor commands (FWD, BWD, LEFT, RIGHT, TURN)
    - SYNC/calibration command
    - Encoder read/reset commands
    - Response parsing
    - Error handling

Usage:
    # Run as pytest
    pytest test/test_arduino.py -v -s

    # Run standalone with GUI
    python3 test/test_arduino.py --gui

    # Run standalone tests only
    python3 test/test_arduino.py --standalone
"""

import sys
import os
import time
import argparse
import threading
from typing import Optional, Tuple

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test.utils.debug_logger import DebugLogger, TestResult, LogLevel
from test.utils.hardware_checker import HardwareChecker


# =============================================================================
# Arduino Bridge (Simplified for testing)
# =============================================================================

class ArduinoTestBridge:
    """
    Simplified Arduino bridge for testing.
    Handles serial communication with full debug output.
    """

    BAUD_RATE = 115200
    TIMEOUT = 2.0
    READY_TIMEOUT = 5.0

    # Motor commands (matching rzv2n-arduino-motor-controller firmware)
    # Movement: CMD,speed,ticks (speed>0, ticks>0 required)
    # Velocity:  VEL,vx,vy,wz (-255..255, needs resend < 200ms)
    CMD_FWD = "FWD"
    CMD_BWD = "BWD"
    CMD_LEFT = "LEFT"
    CMD_RIGHT = "RIGHT"
    CMD_TURN_LEFT = "TURN"   # positive ticks = CCW
    CMD_TURN_RIGHT = "TURN"  # negative ticks = CW
    CMD_STOP = "STOP"
    CMD_SYNC = "SYNC"
    CMD_READ = "READ"
    CMD_RESET = "RESET"
    CMD_VEL = "VEL"

    def __init__(self, port: str = "/dev/ttyACM0",
                 logger: Optional[DebugLogger] = None):
        self.port = port
        self.logger = logger or DebugLogger("Arduino")
        self.serial = None
        self.connected = False
        self._lock = threading.Lock()

    def connect(self) -> Tuple[bool, str]:
        """
        Connect to Arduino with full debug output.

        Returns:
            Tuple of (success, message)
        """
        import serial

        self.logger.info(f"Connecting to Arduino on {self.port}")
        self.logger.debug(f"Baud rate: {self.BAUD_RATE}")
        self.logger.debug(f"Timeout: {self.TIMEOUT}s")

        try:
            # Open serial port
            self.logger.debug("Opening serial port...")
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.BAUD_RATE,
                timeout=self.TIMEOUT
            )
            self.logger.debug("Serial port opened")

            # Wait for Arduino reset (it resets on serial connect)
            self.logger.debug("Waiting for Arduino reset (2s)...")
            time.sleep(2.0)

            # Clear any buffered data
            self.logger.debug("Clearing input buffer...")
            self.serial.reset_input_buffer()

            # Wait for READY message
            self.logger.info("Waiting for READY message...")
            start_time = time.time()
            ready_received = False

            while time.time() - start_time < self.READY_TIMEOUT:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    self.logger.debug(f"Received: '{line}'")
                    if "READY" in line.upper():
                        ready_received = True
                        self.logger.success("READY message received!")
                        break
                time.sleep(0.1)

            if not ready_received:
                self.logger.warning("No READY message (Arduino may still work)")

            self.connected = True
            self.logger.success(f"Connected to Arduino on {self.port}")
            return True, "Connected successfully"

        except serial.SerialException as e:
            self.logger.error(f"Serial error: {e}")
            return False, f"Serial error: {e}"
        except Exception as e:
            self.logger.error(f"Connection error: {e}")
            return False, f"Connection error: {e}"

    def disconnect(self):
        """Disconnect from Arduino."""
        self.logger.info("Disconnecting from Arduino...")
        if self.serial:
            try:
                # Send stop command before disconnecting
                self.send_command(self.CMD_STOP)
                time.sleep(0.1)
                self.serial.close()
                self.logger.success("Disconnected")
            except Exception as e:
                self.logger.error(f"Disconnect error: {e}")
        self.serial = None
        self.connected = False

    def send_velocity(self, vx: int, vy: int, wz: int) -> Tuple[bool, str]:
        """Send VEL command for continuous mecanum control."""
        return self.send_command(self.CMD_VEL, vx, vy, wz)

    def send_command(self, cmd: str, *args) -> Tuple[bool, str]:
        """
        Send command to Arduino with debug output.

        Protocol (matches rzv2n-arduino-motor-controller):
          Movement: CMD,speed,ticks (speed>0, ticks>0 required!)
          Velocity: VEL,vx,vy,wz   (-255..255)
          Simple:   STOP / READ / SYNC

        Returns:
            Tuple of (success, response)
        """
        if not self.connected or not self.serial:
            return False, "Not connected"

        with self._lock:
            try:
                # Format command
                if cmd in [self.CMD_SYNC, self.CMD_READ, self.CMD_RESET, self.CMD_STOP]:
                    command = f"{cmd}\n"
                elif args:
                    command = f"{cmd},{','.join(str(a) for a in args)}\n"
                else:
                    command = f"{cmd}\n"

                self.logger.debug(f"Sending: {command.strip()}")

                # Send command
                self.serial.write(command.encode('utf-8'))
                self.serial.flush()

                # Read response (with timeout)
                time.sleep(0.05)  # Small delay for Arduino to process
                response_lines = []
                start_time = time.time()

                while time.time() - start_time < 0.5:
                    if self.serial.in_waiting:
                        line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            response_lines.append(line)
                            self.logger.debug(f"Response: {line}")
                    else:
                        if response_lines:
                            break
                        time.sleep(0.01)

                response = "\n".join(response_lines) if response_lines else "No response"
                return True, response

            except Exception as e:
                self.logger.error(f"Command error: {e}")
                return False, f"Error: {e}"

    def test_motors(self, duration: float = 0.5, speed: int = 80) -> TestResult:
        """
        Test all motor directions using VEL command for continuous control.

        Args:
            duration: How long to run each test (seconds)
            speed: Motor speed (20-255)

        Returns:
            TestResult with all motor test results
        """
        result = TestResult("Motor Test")
        speed = max(20, min(255, speed))

        # Using VEL,vx,vy,wz for continuous control
        commands = [
            ("Forward",      (speed, 0, 0)),
            ("Backward",     (-speed, 0, 0)),
            ("Strafe Left",  (0, -speed, 0)),
            ("Strafe Right", (0, speed, 0)),
            ("Turn Left",    (0, 0, -speed)),
            ("Turn Right",   (0, 0, speed)),
        ]

        for name, (vx, vy, wz) in commands:
            self.logger.info(f"Testing: {name}")
            step_start = time.time()

            # Send VEL command
            success, response = self.send_velocity(vx, vy, wz)
            if not success:
                result.add_step(name, False, time.time() - step_start,
                                f"Command failed: {response}")
                continue

            # Run for duration (resend to beat watchdog if needed)
            elapsed = 0
            while elapsed < duration:
                time.sleep(0.15)
                elapsed = time.time() - step_start
                if elapsed < duration:
                    self.send_velocity(vx, vy, wz)

            # Stop
            self.send_command(self.CMD_STOP)
            time.sleep(0.3)

            result.add_step(name, True, time.time() - step_start,
                            f"VEL({vx},{vy},{wz}): {response}")

        result.finish()
        return result


# =============================================================================
# Pytest Test Cases
# =============================================================================

class TestArduino:
    """Pytest test class for Arduino."""

    @classmethod
    def setup_class(cls):
        """Setup test fixtures."""
        cls.logger = DebugLogger("ArduinoTest", LogLevel.DEBUG)
        cls.checker = HardwareChecker(cls.logger)
        cls.bridge = None

    @classmethod
    def teardown_class(cls):
        """Cleanup after tests."""
        if cls.bridge:
            cls.bridge.disconnect()

    def test_01_hardware_check(self):
        """Test that Arduino hardware is available."""
        self.logger.header("Test: Hardware Check")

        status = self.checker.check_arduino()
        self.logger.data("Device", status.device)
        self.logger.data("Available", status.available)

        if not status.available:
            self.logger.warning(f"Arduino not available: {status.error}")
            # Skip remaining tests if no hardware
            import pytest
            pytest.skip("Arduino hardware not available")

        self.logger.success("Arduino hardware found")
        assert status.available, f"Arduino not available: {status.error}"

    def test_02_connection(self):
        """Test Arduino connection."""
        self.logger.header("Test: Connection")

        if not self.checker.arduino_available:
            import pytest
            pytest.skip("Arduino not available")

        self.__class__.bridge = ArduinoTestBridge(
            self.checker.arduino_device,
            self.logger
        )

        success, message = self.bridge.connect()
        assert success, f"Connection failed: {message}"
        self.logger.success("Connection test passed")

    def test_03_sync_command(self):
        """Test SYNC/calibration command."""
        self.logger.header("Test: SYNC Command")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("Arduino not connected")

        success, response = self.bridge.send_command(ArduinoTestBridge.CMD_SYNC)
        self.logger.data("Response", response)

        assert success, f"SYNC command failed: {response}"
        self.logger.success("SYNC command test passed")

    def test_04_stop_command(self):
        """Test STOP command."""
        self.logger.header("Test: STOP Command")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("Arduino not connected")

        success, response = self.bridge.send_command(ArduinoTestBridge.CMD_STOP)
        self.logger.data("Response", response)

        assert success, f"STOP command failed: {response}"
        self.logger.success("STOP command test passed")

    def test_05_read_encoders(self):
        """Test READ encoder command."""
        self.logger.header("Test: READ Encoders")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("Arduino not connected")

        success, response = self.bridge.send_command(ArduinoTestBridge.CMD_READ)
        self.logger.data("Response", response)

        assert success, f"READ command failed: {response}"
        self.logger.success("READ encoder test passed")

    def test_06_motor_forward(self):
        """Test forward motor command (brief) using VEL."""
        self.logger.header("Test: Forward Motor")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("Arduino not connected")

        # Brief test using VEL for continuous control
        success, response = self.bridge.send_velocity(50, 0, 0)
        self.logger.data("Response", response)

        time.sleep(0.3)
        self.bridge.send_command(ArduinoTestBridge.CMD_STOP)

        assert success, f"FWD command failed: {response}"
        self.logger.success("Forward motor test passed")


# =============================================================================
# Arduino Test GUI
# =============================================================================

def run_gui(bridge: ArduinoTestBridge):
    """Run interactive GTK GUI for Arduino testing."""
    import gi
    gi.require_version('Gtk', '3.0')
    from gi.repository import Gtk, Gdk, GLib

    class ArduinoTestGUI(Gtk.Window):
        """Interactive GUI for testing Arduino motor controller."""

        def __init__(self, bridge: ArduinoTestBridge):
            super().__init__(title="Arduino Motor Test")
            self.bridge = bridge
            self.set_default_size(500, 600)
            self.set_border_width(15)

            # Main layout
            vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=15)
            self.add(vbox)

            # Title
            title = Gtk.Label()
            title.set_markup("<span size='x-large' weight='bold'>Arduino Motor Controller Test</span>")
            vbox.pack_start(title, False, False, 0)

            # Status
            self.status_label = Gtk.Label(label="Status: Disconnected")
            vbox.pack_start(self.status_label, False, False, 0)

            # Connection button
            self.connect_btn = Gtk.Button(label="Connect")
            self.connect_btn.connect("clicked", self.on_connect)
            vbox.pack_start(self.connect_btn, False, False, 0)

            # Speed slider
            speed_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
            speed_box.pack_start(Gtk.Label(label="Speed:"), False, False, 0)
            self.speed_scale = Gtk.Scale.new_with_range(
                Gtk.Orientation.HORIZONTAL, 0, 255, 10
            )
            self.speed_scale.set_value(100)
            self.speed_scale.set_hexpand(True)
            speed_box.pack_start(self.speed_scale, True, True, 0)
            vbox.pack_start(speed_box, False, False, 0)

            # Motor control buttons (grid)
            motor_grid = Gtk.Grid()
            motor_grid.set_row_spacing(10)
            motor_grid.set_column_spacing(10)
            motor_grid.set_halign(Gtk.Align.CENTER)

            buttons = [
                (None, "FWD", None, 0),
                ("TURNL", "STOP", "TURNR", 1),
                (None, "BWD", None, 2),
                ("LEFT", None, "RIGHT", 3),
            ]

            for row_idx, row in enumerate(buttons):
                for col_idx, cmd in enumerate(row):
                    if cmd:
                        btn = Gtk.Button(label=cmd)
                        btn.set_size_request(80, 50)
                        btn.connect("pressed", self.on_motor_pressed, cmd)
                        btn.connect("released", self.on_motor_released)
                        motor_grid.attach(btn, col_idx, row_idx, 1, 1)

            vbox.pack_start(motor_grid, False, False, 10)

            # Command buttons
            cmd_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
            cmd_box.set_halign(Gtk.Align.CENTER)

            for cmd in ["SYNC", "READ", "RESET"]:
                btn = Gtk.Button(label=cmd)
                btn.connect("clicked", self.on_command, cmd)
                cmd_box.pack_start(btn, False, False, 0)

            vbox.pack_start(cmd_box, False, False, 0)

            # Log output
            log_label = Gtk.Label(label="Log Output:")
            log_label.set_halign(Gtk.Align.START)
            vbox.pack_start(log_label, False, False, 0)

            scroll = Gtk.ScrolledWindow()
            scroll.set_min_content_height(200)
            self.log_view = Gtk.TextView()
            self.log_view.set_editable(False)
            self.log_view.set_monospace(True)
            self.log_buffer = self.log_view.get_buffer()
            scroll.add(self.log_view)
            vbox.pack_start(scroll, True, True, 0)

            # Close button
            close_btn = Gtk.Button(label="Close")
            close_btn.connect("clicked", self.on_close)
            vbox.pack_start(close_btn, False, False, 0)

            self.connect("destroy", self.on_close)
            self.connect("key-press-event", self.on_key_press)

            self.log("Arduino Motor Test GUI started")
            self.log("Press Connect to start testing")

        def log(self, message: str):
            """Add message to log."""
            timestamp = time.strftime("%H:%M:%S")
            text = f"[{timestamp}] {message}\n"
            end_iter = self.log_buffer.get_end_iter()
            self.log_buffer.insert(end_iter, text)
            # Scroll to bottom
            GLib.idle_add(self._scroll_to_bottom)

        def _scroll_to_bottom(self):
            adj = self.log_view.get_parent().get_vadjustment()
            adj.set_value(adj.get_upper() - adj.get_page_size())

        def on_connect(self, button):
            if not self.bridge.connected:
                self.log("Connecting to Arduino...")
                success, message = self.bridge.connect()
                if success:
                    self.status_label.set_text("Status: Connected")
                    self.connect_btn.set_label("Disconnect")
                    self.log(f"Connected: {message}")
                else:
                    self.log(f"Connection failed: {message}")
            else:
                self.bridge.disconnect()
                self.status_label.set_text("Status: Disconnected")
                self.connect_btn.set_label("Connect")
                self.log("Disconnected")

        def on_motor_pressed(self, button, cmd):
            if not self.bridge.connected:
                self.log("Not connected!")
                return
            speed = int(self.speed_scale.get_value())
            self.log(f"Motor: {cmd} @ speed {speed}")
            self.bridge.send_command(cmd, speed, 0)

        def on_motor_released(self, button):
            if self.bridge.connected:
                self.bridge.send_command("STOP")
                self.log("Motor: STOP")

        def on_command(self, button, cmd):
            if not self.bridge.connected:
                self.log("Not connected!")
                return
            self.log(f"Command: {cmd}")
            success, response = self.bridge.send_command(cmd)
            self.log(f"Response: {response}")

        def on_key_press(self, widget, event):
            key = Gdk.keyval_name(event.keyval).lower()
            if key in ('q', 'escape'):
                self.on_close(None)
            return True

        def on_close(self, widget):
            if self.bridge.connected:
                self.bridge.disconnect()
            Gtk.main_quit()

    gui = ArduinoTestGUI(bridge)
    gui.show_all()
    Gtk.main()


# =============================================================================
# Standalone Test Runner
# =============================================================================

def run_standalone_tests():
    """Run tests without pytest."""
    logger = DebugLogger("ArduinoTest", LogLevel.DEBUG)
    checker = HardwareChecker(logger)

    logger.header("Arduino Motor Controller Tests")

    # Check hardware
    logger.subheader("Hardware Check")
    status = checker.check_arduino()
    if not status.available:
        logger.error(f"Arduino not available: {status.error}")
        logger.info("Connect Arduino and try again")
        return False

    logger.success(f"Arduino found on {status.device}")

    # Create bridge
    bridge = ArduinoTestBridge(status.device, logger)

    # Test connection
    logger.subheader("Connection Test")
    success, message = bridge.connect()
    if not success:
        logger.error(f"Connection failed: {message}")
        return False

    # Test commands
    logger.subheader("Command Tests")

    tests = [
        ("SYNC", lambda: bridge.send_command(ArduinoTestBridge.CMD_SYNC)),
        ("STOP", lambda: bridge.send_command(ArduinoTestBridge.CMD_STOP)),
        ("READ", lambda: bridge.send_command(ArduinoTestBridge.CMD_READ)),
    ]

    for name, test_func in tests:
        logger.info(f"Testing {name}...")
        success, response = test_func()
        if success:
            logger.success(f"{name}: {response}")
        else:
            logger.error(f"{name} failed: {response}")

    # Motor test (brief)
    logger.subheader("Motor Test (Brief)")
    result = bridge.test_motors(duration=0.3, speed=60)
    logger.report(result)

    # Cleanup
    bridge.disconnect()

    logger.header("Tests Complete")
    return True


# =============================================================================
# Main Entry Point
# =============================================================================

class MockArduinoBridge:
    """Mock Arduino bridge for testing GUI without hardware."""

    def __init__(self, logger=None):
        self.logger = logger or DebugLogger("MockArduino")
        self.connected = False
        self.speed = 0

    def connect(self):
        self.connected = True
        self.logger.success("Mock Arduino connected")
        return True, "Mock connected"

    def disconnect(self):
        self.connected = False
        self.logger.info("Mock Arduino disconnected")

    def send_command(self, cmd, speed=100, ticks=0):
        self.logger.debug(f"Mock command: {cmd} speed={speed} ticks={ticks}")
        if cmd == "READ":
            return True, "ENC:100,100,100,100"
        return True, f"OK:{cmd}"


def main():
    parser = argparse.ArgumentParser(description="Arduino Motor Controller Tests")
    parser.add_argument("--gui", action="store_true",
                        help="Run interactive GUI")
    parser.add_argument("--standalone", action="store_true",
                        help="Run standalone tests (no pytest)")
    parser.add_argument("--mock", action="store_true",
                        help="Use mock Arduino (no hardware required)")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Arduino serial port")
    args = parser.parse_args()

    if args.gui:
        logger = DebugLogger("ArduinoGUI")

        if args.mock:
            logger.info("Using mock Arduino (no hardware)")
            bridge = MockArduinoBridge(logger)
            run_gui(bridge)
        else:
            checker = HardwareChecker(logger)

            if not checker.arduino_available:
                logger.error("Arduino not available")
                logger.info("Connect Arduino or use --mock flag")
                sys.exit(1)

            bridge = ArduinoTestBridge(checker.arduino_device, logger)
            run_gui(bridge)

    elif args.standalone:
        success = run_standalone_tests()
        sys.exit(0 if success else 1)

    else:
        # Run pytest
        import pytest
        sys.exit(pytest.main([__file__, "-v", "-s"]))


if __name__ == "__main__":
    main()
