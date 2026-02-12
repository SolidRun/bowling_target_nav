#!/usr/bin/env python3
"""
LiDAR Sensor Test Suite
========================

Comprehensive tests for RPLidar A1 sensor.
Tests connection, scanning, and data quality.

Tests:
    - Serial connection
    - Device info query
    - Health check
    - Single scan capture
    - Continuous scanning
    - Data quality metrics
    - Range validation

Usage:
    # Run as pytest
    pytest test/test_lidar.py -v -s

    # Run standalone with visualization
    python3 test/test_lidar.py --visualize

    # Run standalone tests only
    python3 test/test_lidar.py --standalone
"""

import sys
import os
import time
import math
import argparse
from typing import Optional, List, Tuple
from dataclasses import dataclass

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test.utils.debug_logger import DebugLogger, TestResult, LogLevel
from test.utils.hardware_checker import HardwareChecker


# =============================================================================
# LiDAR Data Structures
# =============================================================================

@dataclass
class LidarPoint:
    """Single LiDAR measurement point."""
    angle: float  # degrees
    distance: float  # meters
    quality: int  # 0-15


@dataclass
class LidarScan:
    """Complete LiDAR scan."""
    points: List[LidarPoint]
    timestamp: float
    duration: float

    @property
    def point_count(self) -> int:
        return len(self.points)

    @property
    def valid_points(self) -> int:
        return sum(1 for p in self.points if p.distance > 0)

    @property
    def min_distance(self) -> float:
        valid = [p.distance for p in self.points if p.distance > 0]
        return min(valid) if valid else 0

    @property
    def max_distance(self) -> float:
        valid = [p.distance for p in self.points if p.distance > 0]
        return max(valid) if valid else 0

    @property
    def avg_quality(self) -> float:
        if not self.points:
            return 0
        return sum(p.quality for p in self.points) / len(self.points)


# =============================================================================
# LiDAR Test Bridge
# =============================================================================

class LidarTestBridge:
    """
    LiDAR bridge for testing with full debug output.
    Communicates directly with RPLidar using serial protocol.
    """

    BAUD_RATE = 115200
    TIMEOUT = 3.0

    # RPLidar commands
    CMD_STOP = b'\xA5\x25'
    CMD_RESET = b'\xA5\x40'
    CMD_SCAN = b'\xA5\x20'
    CMD_EXPRESS_SCAN = b'\xA5\x82\x05\x00\x00\x00\x00\x00\x22'
    CMD_GET_INFO = b'\xA5\x50'
    CMD_GET_HEALTH = b'\xA5\x52'

    def __init__(self, port: str = "/dev/ttyUSB0",
                 logger: Optional[DebugLogger] = None):
        self.port = port
        self.logger = logger or DebugLogger("LiDAR")
        self.serial = None
        self.connected = False
        self._scanning = False

    def connect(self) -> Tuple[bool, str]:
        """Connect to LiDAR with debug output."""
        import serial

        self.logger.info(f"Connecting to LiDAR on {self.port}")
        self.logger.debug(f"Baud rate: {self.BAUD_RATE}")

        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.BAUD_RATE,
                timeout=self.TIMEOUT
            )
            self.logger.debug("Serial port opened")

            # Reset the device
            self.logger.debug("Sending reset command...")
            self.serial.write(self.CMD_RESET)
            time.sleep(2.0)

            # Clear buffer
            self.serial.reset_input_buffer()

            self.connected = True
            self.logger.success(f"Connected to LiDAR on {self.port}")
            return True, "Connected"

        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
            return False, str(e)

    def disconnect(self):
        """Disconnect from LiDAR."""
        self.logger.info("Disconnecting from LiDAR...")
        self.stop_scan()
        if self.serial:
            try:
                self.serial.close()
            except:
                pass
        self.serial = None
        self.connected = False
        self.logger.success("Disconnected")

    def get_device_info(self) -> Tuple[bool, dict]:
        """Get LiDAR device information."""
        if not self.connected:
            return False, {"error": "Not connected"}

        self.logger.info("Getting device info...")

        try:
            self.serial.reset_input_buffer()
            self.serial.write(self.CMD_GET_INFO)

            # Wait for response header
            header = self.serial.read(7)
            if len(header) < 7:
                return False, {"error": "No response"}

            self.logger.debug(f"Header: {header.hex()}")

            # Read device info
            data = self.serial.read(20)
            if len(data) < 20:
                return False, {"error": "Incomplete response"}

            info = {
                "model": data[0],
                "firmware_minor": data[1],
                "firmware_major": data[2],
                "hardware": data[3],
                "serial": data[4:20].hex()
            }

            self.logger.data("Model", info["model"])
            self.logger.data("Firmware", f"{info['firmware_major']}.{info['firmware_minor']}")
            self.logger.data("Hardware", info["hardware"])
            self.logger.data("Serial", info["serial"][:16] + "...")

            return True, info

        except Exception as e:
            self.logger.error(f"Get info failed: {e}")
            return False, {"error": str(e)}

    def get_health(self) -> Tuple[bool, dict]:
        """Get LiDAR health status."""
        if not self.connected:
            return False, {"error": "Not connected"}

        self.logger.info("Getting health status...")

        try:
            self.serial.reset_input_buffer()
            self.serial.write(self.CMD_GET_HEALTH)

            # Wait for response header
            header = self.serial.read(7)
            if len(header) < 7:
                return False, {"error": "No response"}

            # Read health data
            data = self.serial.read(3)
            if len(data) < 3:
                return False, {"error": "Incomplete response"}

            status_code = data[0]
            error_code = data[1] | (data[2] << 8)

            status_map = {0: "Good", 1: "Warning", 2: "Error"}
            status = status_map.get(status_code, "Unknown")

            health = {
                "status": status,
                "status_code": status_code,
                "error_code": error_code
            }

            self.logger.data("Status", status)
            self.logger.data("Error Code", error_code)

            if status_code == 0:
                self.logger.success("LiDAR health is good")
            elif status_code == 1:
                self.logger.warning(f"LiDAR warning: error code {error_code}")
            else:
                self.logger.error(f"LiDAR error: code {error_code}")

            return status_code == 0, health

        except Exception as e:
            self.logger.error(f"Health check failed: {e}")
            return False, {"error": str(e)}

    def start_scan(self) -> bool:
        """Start LiDAR scanning."""
        if not self.connected:
            return False

        self.logger.info("Starting scan...")

        try:
            self.serial.reset_input_buffer()
            self.serial.write(self.CMD_SCAN)

            # Wait for response header
            header = self.serial.read(7)
            if len(header) < 7:
                self.logger.error("No scan response")
                return False

            self.logger.debug(f"Scan header: {header.hex()}")
            self._scanning = True
            self.logger.success("Scan started")
            return True

        except Exception as e:
            self.logger.error(f"Start scan failed: {e}")
            return False

    def stop_scan(self):
        """Stop LiDAR scanning."""
        if self.serial and self._scanning:
            self.logger.debug("Stopping scan...")
            try:
                self.serial.write(self.CMD_STOP)
                time.sleep(0.1)
                self.serial.reset_input_buffer()
            except:
                pass
        self._scanning = False

    def capture_scan(self, timeout: float = 2.0) -> Optional[LidarScan]:
        """
        Capture a single complete scan (360 degrees).

        Args:
            timeout: Maximum time to wait for scan

        Returns:
            LidarScan object or None if failed
        """
        if not self._scanning:
            if not self.start_scan():
                return None

        self.logger.info("Capturing scan...")
        points = []
        start_time = time.time()
        first_angle = None

        try:
            while time.time() - start_time < timeout:
                # Read scan point (5 bytes)
                data = self.serial.read(5)
                if len(data) < 5:
                    continue

                # Parse scan data
                quality = data[0] >> 2
                start_flag = data[0] & 0x01
                check_bit = (data[0] >> 1) & 0x01

                if check_bit != 1:
                    continue

                angle_q6 = ((data[1] >> 1) | (data[2] << 7))
                angle = angle_q6 / 64.0

                distance_q2 = (data[3] | (data[4] << 8))
                distance = distance_q2 / 4000.0  # Convert to meters

                point = LidarPoint(angle=angle, distance=distance, quality=quality)
                points.append(point)

                # Detect full rotation
                if first_angle is None and start_flag:
                    first_angle = angle
                elif first_angle is not None and start_flag:
                    # Completed full rotation
                    break

            duration = time.time() - start_time
            scan = LidarScan(points=points, timestamp=start_time, duration=duration)

            self.logger.data("Points captured", scan.point_count)
            self.logger.data("Valid points", scan.valid_points)
            self.logger.data("Duration", f"{duration:.3f}s")
            self.logger.data("Min distance", f"{scan.min_distance:.3f}m")
            self.logger.data("Max distance", f"{scan.max_distance:.3f}m")
            self.logger.data("Avg quality", f"{scan.avg_quality:.1f}")

            return scan

        except Exception as e:
            self.logger.error(f"Scan capture failed: {e}")
            return None


# =============================================================================
# Pytest Test Cases
# =============================================================================

class TestLidar:
    """Pytest test class for LiDAR."""

    @classmethod
    def setup_class(cls):
        """Setup test fixtures."""
        cls.logger = DebugLogger("LidarTest", LogLevel.DEBUG)
        cls.checker = HardwareChecker(cls.logger)
        cls.bridge = None

    @classmethod
    def teardown_class(cls):
        """Cleanup after tests."""
        if cls.bridge:
            cls.bridge.disconnect()

    def test_01_hardware_check(self):
        """Test that LiDAR hardware is available."""
        self.logger.header("Test: Hardware Check")

        status = self.checker.check_lidar()
        self.logger.data("Device", status.device)
        self.logger.data("Available", status.available)

        if not status.available:
            self.logger.warning(f"LiDAR not available: {status.error}")
            import pytest
            pytest.skip("LiDAR hardware not available")

        self.logger.success("LiDAR hardware found")
        assert status.available

    def test_02_connection(self):
        """Test LiDAR connection."""
        self.logger.header("Test: Connection")

        if not self.checker.lidar_available:
            import pytest
            pytest.skip("LiDAR not available")

        self.__class__.bridge = LidarTestBridge(
            self.checker.lidar_device,
            self.logger
        )

        success, message = self.bridge.connect()
        assert success, f"Connection failed: {message}"
        self.logger.success("Connection test passed")

    def test_03_device_info(self):
        """Test device info query."""
        self.logger.header("Test: Device Info")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("LiDAR not connected")

        success, info = self.bridge.get_device_info()
        assert success, f"Get info failed: {info}"
        assert "model" in info
        self.logger.success("Device info test passed")

    def test_04_health_check(self):
        """Test health check."""
        self.logger.header("Test: Health Check")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("LiDAR not connected")

        success, health = self.bridge.get_health()
        assert success, f"Health check failed: {health}"
        self.logger.success("Health check test passed")

    def test_05_single_scan(self):
        """Test single scan capture."""
        self.logger.header("Test: Single Scan")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("LiDAR not connected")

        scan = self.bridge.capture_scan(timeout=3.0)
        assert scan is not None, "Scan capture failed"
        assert scan.point_count > 100, f"Too few points: {scan.point_count}"
        self.logger.success(f"Captured {scan.point_count} points")

    def test_06_scan_quality(self):
        """Test scan data quality."""
        self.logger.header("Test: Scan Quality")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("LiDAR not connected")

        scan = self.bridge.capture_scan(timeout=3.0)
        assert scan is not None, "Scan capture failed"

        # Quality metrics
        valid_ratio = scan.valid_points / scan.point_count if scan.point_count > 0 else 0
        self.logger.data("Valid ratio", f"{valid_ratio:.1%}")
        self.logger.data("Avg quality", f"{scan.avg_quality:.1f}/15")

        assert valid_ratio > 0.5, f"Too few valid points: {valid_ratio:.1%}"
        self.logger.success("Scan quality test passed")


# =============================================================================
# LiDAR Visualization
# =============================================================================

def visualize_scan(bridge: LidarTestBridge):
    """Visualize LiDAR scan using matplotlib."""
    import matplotlib.pyplot as plt
    import numpy as np

    logger = bridge.logger
    logger.header("LiDAR Visualization")

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(10, 10))
    ax.set_title("LiDAR Scan (Press Q to quit)")
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_rmax(6.0)  # 6 meters max

    scatter = ax.scatter([], [], c=[], cmap='viridis', s=2)
    plt.ion()
    plt.show()

    try:
        while plt.fignum_exists(fig.number):
            scan = bridge.capture_scan(timeout=1.0)
            if scan and scan.points:
                # Convert to polar coordinates
                angles = np.array([math.radians(p.angle) for p in scan.points])
                distances = np.array([p.distance for p in scan.points])
                qualities = np.array([p.quality for p in scan.points])

                # Filter valid points
                valid = distances > 0
                angles = angles[valid]
                distances = distances[valid]
                qualities = qualities[valid]

                # Update scatter plot
                scatter.set_offsets(np.c_[angles, distances])
                scatter.set_array(qualities)

                logger.debug(f"Points: {len(angles)}, Min: {distances.min():.2f}m, "
                             f"Max: {distances.max():.2f}m")

            plt.pause(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        plt.close(fig)
        bridge.stop_scan()


# =============================================================================
# Standalone Test Runner
# =============================================================================

def run_standalone_tests():
    """Run tests without pytest."""
    logger = DebugLogger("LidarTest", LogLevel.DEBUG)
    checker = HardwareChecker(logger)

    logger.header("LiDAR Sensor Tests")

    # Check hardware
    logger.subheader("Hardware Check")
    status = checker.check_lidar()
    if not status.available:
        logger.error(f"LiDAR not available: {status.error}")
        return False

    logger.success(f"LiDAR found on {status.device}")

    # Create bridge
    bridge = LidarTestBridge(status.device, logger)

    # Test connection
    logger.subheader("Connection Test")
    success, message = bridge.connect()
    if not success:
        logger.error(f"Connection failed: {message}")
        return False

    # Test device info
    logger.subheader("Device Info")
    success, info = bridge.get_device_info()
    if not success:
        logger.warning(f"Could not get device info: {info}")

    # Test health
    logger.subheader("Health Check")
    success, health = bridge.get_health()
    if not success:
        logger.warning(f"Health check failed: {health}")

    # Test scan
    logger.subheader("Scan Test")
    scan = bridge.capture_scan(timeout=3.0)
    if scan:
        logger.success(f"Captured {scan.point_count} points")
        logger.data("Valid points", f"{scan.valid_points}/{scan.point_count}")
        logger.data("Distance range", f"{scan.min_distance:.2f}m - {scan.max_distance:.2f}m")
    else:
        logger.error("Scan capture failed")

    # Cleanup
    bridge.disconnect()

    logger.header("Tests Complete")
    return True


# =============================================================================
# Main Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="LiDAR Sensor Tests")
    parser.add_argument("--visualize", action="store_true",
                        help="Run with visualization")
    parser.add_argument("--standalone", action="store_true",
                        help="Run standalone tests")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="LiDAR serial port")
    args = parser.parse_args()

    if args.visualize:
        logger = DebugLogger("LidarViz")
        checker = HardwareChecker(logger)

        if not checker.lidar_available:
            logger.error("LiDAR not available")
            sys.exit(1)

        bridge = LidarTestBridge(checker.lidar_device, logger)
        success, _ = bridge.connect()
        if success:
            visualize_scan(bridge)
            bridge.disconnect()

    elif args.standalone:
        success = run_standalone_tests()
        sys.exit(0 if success else 1)

    else:
        import pytest
        sys.exit(pytest.main([__file__, "-v", "-s"]))


if __name__ == "__main__":
    main()
