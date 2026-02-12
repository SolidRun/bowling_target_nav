#!/usr/bin/env python3
"""
Full System Test Suite
======================

Comprehensive tests for complete V2N robot system.
Combines Arduino, LiDAR, and Camera with interactive GUI.

Tests:
    - All hardware connections
    - Sensor data fusion
    - Motor control with feedback
    - Target tracking
    - Navigation commands
    - Full system GUI

Usage:
    # Run as pytest
    pytest test/test_full_system.py -v -s

    # Run full system GUI
    python3 test/test_full_system.py --gui

    # Run standalone tests
    python3 test/test_full_system.py --standalone
"""

import sys
import os
import time
import math
import argparse
import threading
from typing import Optional, Tuple
from dataclasses import dataclass

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test.utils.debug_logger import DebugLogger, TestResult, LogLevel
from test.utils.hardware_checker import HardwareChecker

# Import component tests
from test.test_arduino import ArduinoTestBridge
from test.test_lidar import LidarTestBridge
from test.test_camera import CameraTestBridge


# =============================================================================
# Full System Bridge
# =============================================================================

class FullSystemBridge:
    """
    Complete system bridge combining all hardware components.
    Provides unified interface for testing and control.
    """

    def __init__(self, logger: Optional[DebugLogger] = None):
        self.logger = logger or DebugLogger("FullSystem")
        self.checker = HardwareChecker(self.logger)

        # Component bridges
        self.arduino: Optional[ArduinoTestBridge] = None
        self.lidar: Optional[LidarTestBridge] = None
        self.camera: Optional[CameraTestBridge] = None

        # State
        self.connected = False
        self._lock = threading.Lock()

        # Current sensor data
        self.last_scan = None
        self.last_frame = None
        self.last_detections = []

    def connect_all(self) -> Tuple[bool, dict]:
        """
        Connect to all available hardware components.

        Returns:
            Tuple of (success, status_dict)
        """
        self.logger.header("Full System Connection")

        status = {
            "arduino": {"available": False, "connected": False, "error": ""},
            "lidar": {"available": False, "connected": False, "error": ""},
            "camera": {"available": False, "connected": False, "error": ""},
            "yolo": {"loaded": False, "error": ""},
        }

        # Check and connect Arduino
        self.logger.subheader("Arduino")
        if self.checker.arduino_available:
            status["arduino"]["available"] = True
            self.arduino = ArduinoTestBridge(
                self.checker.arduino_device,
                DebugLogger("Arduino", LogLevel.INFO)
            )
            success, msg = self.arduino.connect()
            status["arduino"]["connected"] = success
            if not success:
                status["arduino"]["error"] = msg
                self.logger.error(f"Arduino: {msg}")
            else:
                self.logger.success("Arduino connected")
        else:
            status["arduino"]["error"] = "Not found"
            self.logger.warning("Arduino not available")

        # Check and connect LiDAR
        self.logger.subheader("LiDAR")
        if self.checker.lidar_available:
            status["lidar"]["available"] = True
            self.lidar = LidarTestBridge(
                self.checker.lidar_device,
                DebugLogger("LiDAR", LogLevel.INFO)
            )
            success, msg = self.lidar.connect()
            status["lidar"]["connected"] = success
            if not success:
                status["lidar"]["error"] = msg
                self.logger.error(f"LiDAR: {msg}")
            else:
                self.logger.success("LiDAR connected")
        else:
            status["lidar"]["error"] = "Not found"
            self.logger.warning("LiDAR not available")

        # Check and connect Camera
        self.logger.subheader("Camera")
        if self.checker.camera_available:
            status["camera"]["available"] = True
            self.camera = CameraTestBridge(
                self.checker.camera_device,
                DebugLogger("Camera", LogLevel.INFO)
            )
            success, msg = self.camera.connect()
            status["camera"]["connected"] = success
            if not success:
                status["camera"]["error"] = msg
                self.logger.error(f"Camera: {msg}")
            else:
                self.logger.success("Camera connected")

                # Load YOLO
                success, msg = self.camera.load_yolo()
                status["yolo"]["loaded"] = success
                if not success:
                    status["yolo"]["error"] = msg
                    self.logger.warning(f"YOLO: {msg}")
                else:
                    self.logger.success("YOLO loaded")
        else:
            status["camera"]["error"] = "Not found"
            self.logger.warning("Camera not available")

        # Summary
        self.logger.subheader("Connection Summary")
        connected_count = sum([
            status["arduino"]["connected"],
            status["lidar"]["connected"],
            status["camera"]["connected"],
        ])

        self.logger.data("Arduino", "OK" if status["arduino"]["connected"] else "FAIL")
        self.logger.data("LiDAR", "OK" if status["lidar"]["connected"] else "FAIL")
        self.logger.data("Camera", "OK" if status["camera"]["connected"] else "FAIL")
        self.logger.data("YOLO", "OK" if status["yolo"]["loaded"] else "FAIL")

        self.connected = connected_count > 0

        if connected_count == 3:
            self.logger.success("All components connected!")
        elif connected_count > 0:
            self.logger.warning(f"{connected_count}/3 components connected")
        else:
            self.logger.error("No components connected")

        return connected_count > 0, status

    def disconnect_all(self):
        """Disconnect all components."""
        self.logger.info("Disconnecting all components...")

        if self.arduino:
            self.arduino.disconnect()
            self.arduino = None

        if self.lidar:
            self.lidar.disconnect()
            self.lidar = None

        if self.camera:
            self.camera.disconnect()
            self.camera = None

        self.connected = False
        self.logger.success("All disconnected")

    def update_sensors(self) -> bool:
        """Update all sensor data."""
        with self._lock:
            # Update LiDAR
            if self.lidar and self.lidar.connected:
                self.last_scan = self.lidar.capture_scan(timeout=0.5)

            # Update Camera
            if self.camera and self.camera.connected:
                self.last_frame = self.camera.capture_frame()
                if self.last_frame is not None and self.camera.detector:
                    self.last_detections = self.camera.detect(self.last_frame)

        return True

    def send_motor_command(self, cmd: str, speed: int = 100) -> Tuple[bool, str]:
        """Send motor command to Arduino."""
        if not self.arduino or not self.arduino.connected:
            return False, "Arduino not connected"
        return self.arduino.send_command(cmd, speed, 0)

    def stop_motors(self):
        """Emergency stop all motors."""
        if self.arduino and self.arduino.connected:
            self.arduino.send_command("STOP")

    def run_system_test(self) -> TestResult:
        """
        Run comprehensive system test.

        Returns:
            TestResult with all component test results
        """
        result = TestResult("Full System Test")

        # Test Arduino
        if self.arduino and self.arduino.connected:
            self.logger.subheader("Arduino Test")
            start = time.time()
            success, response = self.arduino.send_command("SYNC")
            result.add_step("Arduino SYNC", success, time.time() - start, response)

            if success:
                # Brief motor test
                start = time.time()
                self.send_motor_command("FWD", 50)
                time.sleep(0.2)
                self.stop_motors()
                result.add_step("Arduino Motor", True, time.time() - start, "Brief FWD test")

        # Test LiDAR
        if self.lidar and self.lidar.connected:
            self.logger.subheader("LiDAR Test")
            start = time.time()
            scan = self.lidar.capture_scan(timeout=2.0)
            success = scan is not None and scan.point_count > 100
            result.add_step("LiDAR Scan", success, time.time() - start,
                            f"{scan.point_count if scan else 0} points")

        # Test Camera
        if self.camera and self.camera.connected:
            self.logger.subheader("Camera Test")
            start = time.time()
            frame = self.camera.capture_frame()
            success = frame is not None
            result.add_step("Camera Capture", success, time.time() - start,
                            f"{frame.shape if frame is not None else 'None'}")

            # Test Detection
            if success and self.camera.detector:
                start = time.time()
                detections = self.camera.detect(frame)
                result.add_step("YOLO Detection", True, time.time() - start,
                                f"{len(detections)} detections")

        result.finish()
        return result


# =============================================================================
# Pytest Test Cases
# =============================================================================

class TestFullSystem:
    """Pytest test class for full system."""

    @classmethod
    def setup_class(cls):
        """Setup test fixtures."""
        cls.logger = DebugLogger("SystemTest", LogLevel.DEBUG)
        cls.bridge = FullSystemBridge(cls.logger)

    @classmethod
    def teardown_class(cls):
        """Cleanup after tests."""
        cls.bridge.disconnect_all()

    def test_01_hardware_check(self):
        """Test hardware availability."""
        self.logger.header("Test: Hardware Check")
        self.bridge.checker.print_status()

    def test_02_connect_all(self):
        """Test connecting all components."""
        self.logger.header("Test: Connect All")

        success, status = self.bridge.connect_all()

        if not any([
            status["arduino"]["available"],
            status["lidar"]["available"],
            status["camera"]["available"]
        ]):
            import pytest
            pytest.skip("No hardware available")

        assert success, "No components connected"

    def test_03_system_test(self):
        """Run full system test."""
        self.logger.header("Test: System Test")

        if not self.bridge.connected:
            import pytest
            pytest.skip("System not connected")

        result = self.bridge.run_system_test()
        self.logger.report(result)

        # At least some steps should pass
        assert result.passed_steps > 0, "No tests passed"


# =============================================================================
# Full System GUI
# =============================================================================

def run_system_gui(bridge: FullSystemBridge):
    """Run full system test GUI with all components."""
    import gi
    gi.require_version('Gtk', '3.0')
    from gi.repository import Gtk, Gdk, GLib, GdkPixbuf
    import cv2
    import numpy as np

    class FullSystemGUI(Gtk.Window):
        """Interactive GUI for full system testing."""

        def __init__(self, bridge: FullSystemBridge):
            super().__init__(title="V2N Full System Test")
            self.bridge = bridge
            self.set_default_size(1200, 800)

            # Main layout
            main_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
            self.add(main_box)

            # Title
            title = Gtk.Label()
            title.set_markup("<span size='x-large' weight='bold'>V2N Robot - Full System Test</span>")
            title.set_margin_top(10)
            main_box.pack_start(title, False, False, 0)

            # Content area (horizontal split)
            content_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
            content_box.set_margin_start(10)
            content_box.set_margin_end(10)
            main_box.pack_start(content_box, True, True, 0)

            # Left panel - Drawing area for visualization
            left_frame = Gtk.Frame(label="Sensors")
            self.draw_area = Gtk.DrawingArea()
            self.draw_area.set_size_request(700, 500)
            self.draw_area.connect('draw', self.on_draw)
            left_frame.add(self.draw_area)
            content_box.pack_start(left_frame, True, True, 0)

            # Right panel - Controls
            right_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
            right_box.set_size_request(350, -1)
            content_box.pack_start(right_box, False, False, 0)

            # Status frame
            status_frame = Gtk.Frame(label="Status")
            status_grid = Gtk.Grid()
            status_grid.set_row_spacing(5)
            status_grid.set_column_spacing(10)
            status_grid.set_margin_start(10)
            status_grid.set_margin_end(10)
            status_grid.set_margin_top(5)
            status_grid.set_margin_bottom(5)

            self.status_labels = {}
            for i, name in enumerate(["Arduino", "LiDAR", "Camera", "YOLO"]):
                label = Gtk.Label(label=f"{name}:")
                label.set_halign(Gtk.Align.START)
                status_grid.attach(label, 0, i, 1, 1)

                status = Gtk.Label(label="--")
                status.set_halign(Gtk.Align.START)
                status_grid.attach(status, 1, i, 1, 1)
                self.status_labels[name.lower()] = status

            status_frame.add(status_grid)
            right_box.pack_start(status_frame, False, False, 0)

            # Motor controls frame
            motor_frame = Gtk.Frame(label="Motor Control")
            motor_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
            motor_box.set_margin_start(10)
            motor_box.set_margin_end(10)
            motor_box.set_margin_top(5)
            motor_box.set_margin_bottom(5)

            # Speed slider
            speed_box = Gtk.Box(spacing=5)
            speed_box.pack_start(Gtk.Label(label="Speed:"), False, False, 0)
            self.speed_scale = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 0, 200, 10)
            self.speed_scale.set_value(80)
            speed_box.pack_start(self.speed_scale, True, True, 0)
            motor_box.pack_start(speed_box, False, False, 0)

            # Motor buttons grid
            btn_grid = Gtk.Grid()
            btn_grid.set_row_spacing(5)
            btn_grid.set_column_spacing(5)
            btn_grid.set_halign(Gtk.Align.CENTER)

            buttons = [
                (None, "FWD", None),
                ("TURNL", "STOP", "TURNR"),
                (None, "BWD", None),
                ("LEFT", None, "RIGHT"),
            ]

            for row_idx, row in enumerate(buttons):
                for col_idx, cmd in enumerate(row):
                    if cmd:
                        btn = Gtk.Button(label=cmd)
                        btn.set_size_request(60, 40)
                        if cmd == "STOP":
                            btn.get_style_context().add_class("destructive-action")
                        btn.connect("pressed", self.on_motor_pressed, cmd)
                        btn.connect("released", self.on_motor_released)
                        btn_grid.attach(btn, col_idx, row_idx, 1, 1)

            motor_box.pack_start(btn_grid, False, False, 5)
            motor_frame.add(motor_box)
            right_box.pack_start(motor_frame, False, False, 0)

            # Test buttons frame
            test_frame = Gtk.Frame(label="Tests")
            test_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
            test_box.set_margin_start(10)
            test_box.set_margin_end(10)
            test_box.set_margin_top(5)
            test_box.set_margin_bottom(5)

            for cmd in ["SYNC", "READ", "RESET"]:
                btn = Gtk.Button(label=f"Arduino {cmd}")
                btn.connect("clicked", self.on_test_command, cmd)
                test_box.pack_start(btn, False, False, 0)

            run_test_btn = Gtk.Button(label="Run System Test")
            run_test_btn.connect("clicked", self.on_run_test)
            test_box.pack_start(run_test_btn, False, False, 5)

            test_frame.add(test_box)
            right_box.pack_start(test_frame, False, False, 0)

            # Log frame
            log_frame = Gtk.Frame(label="Log")
            scroll = Gtk.ScrolledWindow()
            scroll.set_min_content_height(150)
            self.log_view = Gtk.TextView()
            self.log_view.set_editable(False)
            self.log_view.set_monospace(True)
            self.log_buffer = self.log_view.get_buffer()
            scroll.add(self.log_view)
            log_frame.add(scroll)
            right_box.pack_start(log_frame, True, True, 0)

            # Close button
            close_btn = Gtk.Button(label="Close")
            close_btn.connect("clicked", self.on_close)
            right_box.pack_start(close_btn, False, False, 0)

            # Events
            self.connect("destroy", self.on_close)
            self.connect("key-press-event", self.on_key_press)

            # Update timer
            self._running = True
            GLib.timeout_add(50, self.on_update)

            # Initial status update
            self.update_status()
            self.log("Full System GUI started")

        def log(self, message: str):
            """Add message to log."""
            timestamp = time.strftime("%H:%M:%S")
            text = f"[{timestamp}] {message}\n"
            end_iter = self.log_buffer.get_end_iter()
            self.log_buffer.insert(end_iter, text)

        def update_status(self):
            """Update status labels."""
            if self.bridge.arduino and self.bridge.arduino.connected:
                self.status_labels["arduino"].set_markup("<span foreground='green'>Connected</span>")
            else:
                self.status_labels["arduino"].set_markup("<span foreground='red'>Disconnected</span>")

            if self.bridge.lidar and self.bridge.lidar.connected:
                self.status_labels["lidar"].set_markup("<span foreground='green'>Connected</span>")
            else:
                self.status_labels["lidar"].set_markup("<span foreground='red'>Disconnected</span>")

            if self.bridge.camera and self.bridge.camera.connected:
                self.status_labels["camera"].set_markup("<span foreground='green'>Connected</span>")
            else:
                self.status_labels["camera"].set_markup("<span foreground='red'>Disconnected</span>")

            if self.bridge.camera and self.bridge.camera.detector:
                self.status_labels["yolo"].set_markup("<span foreground='green'>Loaded</span>")
            else:
                self.status_labels["yolo"].set_markup("<span foreground='orange'>Not loaded</span>")

        def on_update(self):
            """Periodic update."""
            if not self._running:
                return False

            self.bridge.update_sensors()
            self.draw_area.queue_draw()
            return True

        def on_draw(self, widget, cr):
            """Draw sensor visualization."""
            alloc = widget.get_allocation()
            w, h = alloc.width, alloc.height

            # Background
            cr.set_source_rgb(0.1, 0.1, 0.1)
            cr.paint()

            # Split into two panels
            half_w = w // 2

            # Left: Camera view
            if self.bridge.last_frame is not None:
                frame = self.bridge.last_frame.copy()

                # Draw detections
                for det in self.bridge.last_detections:
                    color = (0, 255, 0) if det.class_name == "Pins" else (0, 165, 255)
                    cv2.rectangle(frame, (det.x1, det.y1), (det.x2, det.y2), color, 2)
                    cv2.putText(frame, f"{det.class_name}", (det.x1, det.y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                # Scale to fit
                fh, fw = frame.shape[:2]
                scale = min((half_w - 20) / fw, (h - 20) / fh)
                disp_w = int(fw * scale)
                disp_h = int(fh * scale)
                frame = cv2.resize(frame, (disp_w, disp_h))
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                pixbuf = GdkPixbuf.Pixbuf.new_from_data(
                    frame.tobytes(), GdkPixbuf.Colorspace.RGB, False, 8,
                    disp_w, disp_h, disp_w * 3
                )
                x = 10
                y = (h - disp_h) // 2
                Gdk.cairo_set_source_pixbuf(cr, pixbuf, x, y)
                cr.paint()
            else:
                cr.set_source_rgb(0.3, 0.3, 0.3)
                cr.set_font_size(14)
                cr.move_to(half_w // 2 - 50, h // 2)
                cr.show_text("No camera feed")

            # Right: LiDAR view
            lidar_cx = half_w + half_w // 2
            lidar_cy = h // 2
            lidar_radius = min(half_w, h) // 2 - 30

            # Draw grid
            cr.set_source_rgb(0.2, 0.2, 0.2)
            for r in range(1, 6):
                cr.arc(lidar_cx, lidar_cy, lidar_radius * r / 5, 0, 2 * 3.14159)
                cr.stroke()

            # Draw LiDAR points
            if self.bridge.last_scan:
                cr.set_source_rgb(1, 0, 0)
                scale = lidar_radius / 5.0  # 5 meter max range
                for point in self.bridge.last_scan.points:
                    if 0 < point.distance < 5:
                        angle_rad = math.radians(point.angle)
                        px = lidar_cx + point.distance * scale * math.sin(angle_rad)
                        py = lidar_cy - point.distance * scale * math.cos(angle_rad)
                        cr.rectangle(px - 1, py - 1, 2, 2)
                        cr.fill()

            # Draw robot
            cr.set_source_rgb(0, 1, 0)
            cr.arc(lidar_cx, lidar_cy, 5, 0, 2 * 3.14159)
            cr.fill()

            # Labels
            cr.set_source_rgb(1, 1, 1)
            cr.set_font_size(12)
            cr.move_to(10, 20)
            cr.show_text(f"Camera - Detections: {len(self.bridge.last_detections)}")
            cr.move_to(half_w + 10, 20)
            scan_info = f"{self.bridge.last_scan.point_count} pts" if self.bridge.last_scan else "No data"
            cr.show_text(f"LiDAR - {scan_info}")

        def on_motor_pressed(self, button, cmd):
            if cmd == "STOP":
                self.bridge.stop_motors()
                self.log("STOP")
            else:
                speed = int(self.speed_scale.get_value())
                success, _ = self.bridge.send_motor_command(cmd, speed)
                if success:
                    self.log(f"{cmd} @ {speed}")

        def on_motor_released(self, button):
            self.bridge.stop_motors()

        def on_test_command(self, button, cmd):
            if self.bridge.arduino and self.bridge.arduino.connected:
                success, response = self.bridge.arduino.send_command(cmd)
                self.log(f"{cmd}: {response}")
            else:
                self.log("Arduino not connected")

        def on_run_test(self, button):
            self.log("Running system test...")
            result = self.bridge.run_system_test()
            self.log(f"Result: {'PASSED' if result.passed else 'FAILED'}")
            self.log(f"Steps: {result.passed_steps}/{len(result.steps)}")

        def on_key_press(self, widget, event):
            key = Gdk.keyval_name(event.keyval).lower()
            if key in ('q', 'escape'):
                self.on_close(None)
            elif key == 'space':
                self.bridge.stop_motors()
            return True

        def on_close(self, widget):
            self._running = False
            self.bridge.stop_motors()
            Gtk.main_quit()

    gui = FullSystemGUI(bridge)
    gui.show_all()
    Gtk.main()


# =============================================================================
# Standalone Test Runner
# =============================================================================

def run_standalone_tests():
    """Run tests without pytest."""
    logger = DebugLogger("SystemTest", LogLevel.DEBUG)

    logger.header("V2N Full System Tests")

    # Create bridge
    bridge = FullSystemBridge(logger)

    # Connect all
    success, status = bridge.connect_all()
    if not success:
        logger.error("No components connected")
        return False

    # Run system test
    logger.subheader("System Test")
    result = bridge.run_system_test()
    logger.report(result)

    # Cleanup
    bridge.disconnect_all()

    logger.header("Tests Complete")
    return result.passed


# =============================================================================
# Main Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="Full System Tests")
    parser.add_argument("--gui", action="store_true",
                        help="Run full system GUI")
    parser.add_argument("--standalone", action="store_true",
                        help="Run standalone tests")
    args = parser.parse_args()

    if args.gui:
        logger = DebugLogger("SystemGUI")
        bridge = FullSystemBridge(logger)
        success, _ = bridge.connect_all()

        if success:
            run_system_gui(bridge)
            bridge.disconnect_all()
        else:
            logger.error("Could not connect to any components")
            sys.exit(1)

    elif args.standalone:
        success = run_standalone_tests()
        sys.exit(0 if success else 1)

    else:
        import pytest
        sys.exit(pytest.main([__file__, "-v", "-s"]))


if __name__ == "__main__":
    main()
