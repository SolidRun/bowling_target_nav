#!/usr/bin/env python3
"""
LiDAR + Camera Integration Test Suite
======================================

Integration tests combining LiDAR and camera data.
Tests sensor fusion and obstacle detection near detected objects.

Tests:
    - Concurrent sensor operation
    - Data synchronization
    - Detection with range validation
    - Obstacle detection near targets
    - Combined visualization

Usage:
    # Run as pytest
    pytest test/test_lidar_camera.py -v -s

    # Run standalone with visualization
    python3 test/test_lidar_camera.py --visualize

    # Run standalone tests
    python3 test/test_lidar_camera.py --standalone
"""

import sys
import os
import time
import math
import argparse
import threading
from typing import Optional, List, Tuple
from dataclasses import dataclass

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test.utils.debug_logger import DebugLogger, TestResult, LogLevel
from test.utils.hardware_checker import HardwareChecker

# Import component tests
from test.test_lidar import LidarTestBridge, LidarScan
from test.test_camera import CameraTestBridge, DetectionResult


# =============================================================================
# Combined Sensor Data
# =============================================================================

@dataclass
class SensorFusionData:
    """Combined LiDAR and camera data."""
    timestamp: float
    lidar_scan: Optional[LidarScan]
    camera_frame: any  # numpy array
    detections: List[DetectionResult]

    # Computed fields
    obstacles_in_view: List[Tuple[float, float]]  # (angle, distance)
    target_lidar_distance: Optional[float] = None


# =============================================================================
# Sensor Fusion Bridge
# =============================================================================

class SensorFusionBridge:
    """
    Combined LiDAR + Camera bridge for sensor fusion testing.
    """

    def __init__(self, lidar_port: str = "/dev/ttyUSB0",
                 camera_device: str = "/dev/video0",
                 logger: Optional[DebugLogger] = None):
        self.logger = logger or DebugLogger("SensorFusion")

        self.lidar = LidarTestBridge(lidar_port, DebugLogger("LiDAR", LogLevel.INFO))
        self.camera = CameraTestBridge(camera_device, DebugLogger("Camera", LogLevel.INFO))

        self.connected = False
        self._lock = threading.Lock()

        # Camera FOV parameters (adjust for your camera)
        self.camera_hfov = 60.0  # horizontal field of view in degrees
        self.camera_center_angle = 0.0  # camera pointing direction

    def connect(self) -> Tuple[bool, str]:
        """Connect to both sensors."""
        self.logger.header("Connecting Sensors")

        errors = []

        # Connect LiDAR
        self.logger.subheader("LiDAR Connection")
        success, msg = self.lidar.connect()
        if not success:
            errors.append(f"LiDAR: {msg}")
            self.logger.error(f"LiDAR connection failed: {msg}")
        else:
            self.logger.success("LiDAR connected")

        # Connect Camera
        self.logger.subheader("Camera Connection")
        success, msg = self.camera.connect()
        if not success:
            errors.append(f"Camera: {msg}")
            self.logger.error(f"Camera connection failed: {msg}")
        else:
            self.logger.success("Camera connected")

        # Load YOLO
        self.logger.subheader("YOLO Model")
        success, msg = self.camera.load_yolo()
        if not success:
            self.logger.warning(f"YOLO not available: {msg}")
        else:
            self.logger.success("YOLO loaded")

        if errors:
            return False, "; ".join(errors)

        self.connected = True
        self.logger.success("All sensors connected")
        return True, "Connected"

    def disconnect(self):
        """Disconnect all sensors."""
        self.logger.info("Disconnecting sensors...")
        self.lidar.disconnect()
        self.camera.disconnect()
        self.connected = False
        self.logger.success("All sensors disconnected")

    def capture_fused_data(self) -> Optional[SensorFusionData]:
        """
        Capture synchronized data from both sensors.

        Returns:
            SensorFusionData with combined sensor readings
        """
        if not self.connected:
            return None

        timestamp = time.time()
        self.logger.debug("Capturing fused data...")

        # Capture camera frame and run detection
        frame = self.camera.capture_frame()
        detections = []
        if frame is not None and self.camera.detector:
            detections = self.camera.detect(frame)

        # Capture LiDAR scan
        lidar_scan = self.lidar.capture_scan(timeout=1.0)

        # Find obstacles in camera FOV
        obstacles_in_view = []
        if lidar_scan:
            half_fov = self.camera_hfov / 2
            min_angle = self.camera_center_angle - half_fov
            max_angle = self.camera_center_angle + half_fov

            for point in lidar_scan.points:
                # Normalize angle to -180 to 180
                angle = point.angle
                if angle > 180:
                    angle -= 360

                if min_angle <= angle <= max_angle and point.distance > 0:
                    obstacles_in_view.append((angle, point.distance))

        # Try to match detection with LiDAR distance
        target_lidar_distance = None
        if detections and lidar_scan:
            # Find "Pins" detection
            pins = [d for d in detections if d.class_name == "Pins"]
            if pins:
                best_pin = pins[0]
                # Convert detection angle to LiDAR angle
                # (assuming camera and LiDAR are roughly aligned)
                det_angle = math.degrees(best_pin.angle) if best_pin.angle else 0

                # Find closest LiDAR point at that angle
                angle_tolerance = 10.0  # degrees
                matching_points = [
                    p.distance for p in lidar_scan.points
                    if abs((p.angle if p.angle <= 180 else p.angle - 360) - det_angle) < angle_tolerance
                    and p.distance > 0
                ]
                if matching_points:
                    target_lidar_distance = min(matching_points)

        data = SensorFusionData(
            timestamp=timestamp,
            lidar_scan=lidar_scan,
            camera_frame=frame,
            detections=detections,
            obstacles_in_view=obstacles_in_view,
            target_lidar_distance=target_lidar_distance
        )

        # Log summary
        self.logger.debug(f"Captured: {len(detections)} detections, "
                          f"{len(obstacles_in_view)} obstacles in view")
        if target_lidar_distance:
            self.logger.debug(f"Target LiDAR distance: {target_lidar_distance:.2f}m")

        return data

    def run_concurrent_test(self, duration: float = 5.0) -> TestResult:
        """
        Test concurrent sensor operation.

        Args:
            duration: How long to run the test

        Returns:
            TestResult with timing and data quality metrics
        """
        result = TestResult("Concurrent Sensor Test")
        self.logger.info(f"Running concurrent test for {duration}s...")

        start_time = time.time()
        capture_count = 0
        lidar_count = 0
        camera_count = 0
        detection_count = 0

        while time.time() - start_time < duration:
            data = self.capture_fused_data()
            if data:
                capture_count += 1
                if data.lidar_scan:
                    lidar_count += 1
                if data.camera_frame is not None:
                    camera_count += 1
                detection_count += len(data.detections)

        elapsed = time.time() - start_time

        # Calculate rates
        capture_rate = capture_count / elapsed
        lidar_rate = lidar_count / elapsed
        camera_rate = camera_count / elapsed

        result.add_step("Capture rate", capture_rate > 1.0, elapsed,
                        f"{capture_rate:.1f} Hz")
        result.add_step("LiDAR rate", lidar_rate > 0.5, elapsed,
                        f"{lidar_rate:.1f} Hz")
        result.add_step("Camera rate", camera_rate > 5.0, elapsed,
                        f"{camera_rate:.1f} Hz")

        self.logger.data("Total captures", capture_count)
        self.logger.data("LiDAR scans", lidar_count)
        self.logger.data("Camera frames", camera_count)
        self.logger.data("Total detections", detection_count)
        self.logger.data("Capture rate", f"{capture_rate:.1f} Hz")

        result.finish()
        return result


# =============================================================================
# Pytest Test Cases
# =============================================================================

class TestLidarCamera:
    """Pytest test class for LiDAR + Camera integration."""

    @classmethod
    def setup_class(cls):
        """Setup test fixtures."""
        cls.logger = DebugLogger("IntegrationTest", LogLevel.DEBUG)
        cls.checker = HardwareChecker(cls.logger)
        cls.bridge = None

    @classmethod
    def teardown_class(cls):
        """Cleanup after tests."""
        if cls.bridge:
            cls.bridge.disconnect()

    def test_01_hardware_check(self):
        """Test that both LiDAR and camera are available."""
        self.logger.header("Test: Hardware Check")

        lidar_ok = self.checker.lidar_available
        camera_ok = self.checker.camera_available

        self.logger.data("LiDAR", "OK" if lidar_ok else "Missing")
        self.logger.data("Camera", "OK" if camera_ok else "Missing")

        if not lidar_ok or not camera_ok:
            import pytest
            pytest.skip("LiDAR or Camera not available")

        self.logger.success("Both sensors available")

    def test_02_connection(self):
        """Test connecting to both sensors."""
        self.logger.header("Test: Sensor Connection")

        self.__class__.bridge = SensorFusionBridge(
            self.checker.lidar_device,
            self.checker.camera_device,
            self.logger
        )

        success, message = self.bridge.connect()
        assert success, f"Connection failed: {message}"
        self.logger.success("Both sensors connected")

    def test_03_fused_capture(self):
        """Test capturing fused data."""
        self.logger.header("Test: Fused Data Capture")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("Sensors not connected")

        data = self.bridge.capture_fused_data()
        assert data is not None, "Capture failed"
        assert data.camera_frame is not None, "No camera frame"

        self.logger.data("Detections", len(data.detections))
        self.logger.data("Obstacles in view", len(data.obstacles_in_view))

        self.logger.success("Fused capture test passed")

    def test_04_concurrent_operation(self):
        """Test concurrent sensor operation."""
        self.logger.header("Test: Concurrent Operation")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("Sensors not connected")

        result = self.bridge.run_concurrent_test(duration=3.0)
        self.logger.report(result)

        assert result.passed, "Concurrent operation test failed"

    def test_05_detection_with_range(self):
        """Test detection with LiDAR range validation."""
        self.logger.header("Test: Detection with Range")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("Sensors not connected")

        # Capture multiple frames and look for detection with range
        for i in range(10):
            data = self.bridge.capture_fused_data()
            if data and data.detections and data.target_lidar_distance:
                self.logger.info(f"Detection with LiDAR range found!")
                self.logger.data("Camera distance estimate",
                                 f"{data.detections[0].distance:.2f}m")
                self.logger.data("LiDAR distance",
                                 f"{data.target_lidar_distance:.2f}m")
                self.logger.success("Detection with range test passed")
                return

        self.logger.warning("No detection with LiDAR range found")
        # Don't fail - target may not be in view


# =============================================================================
# Combined Visualization
# =============================================================================

def run_visualization(bridge: SensorFusionBridge):
    """Run combined visualization of LiDAR and camera."""
    import cv2
    import numpy as np

    logger = bridge.logger
    logger.header("Sensor Fusion Visualization")
    logger.info("Press Q to quit")

    cv2.namedWindow("Sensor Fusion", cv2.WINDOW_NORMAL)

    try:
        while True:
            data = bridge.capture_fused_data()
            if data is None or data.camera_frame is None:
                continue

            # Create display frame
            frame = data.camera_frame.copy()
            h, w = frame.shape[:2]

            # Draw detections
            for det in data.detections:
                color = (0, 255, 0) if det.class_name == "Pins" else (0, 165, 255)
                cv2.rectangle(frame, (det.x1, det.y1), (det.x2, det.y2), color, 2)

                label = f"{det.class_name}: {det.confidence:.2f}"
                if det.distance > 0:
                    label += f" (cam: {det.distance:.2f}m)"
                cv2.putText(frame, label, (det.x1, det.y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Add LiDAR distance if available
            if data.target_lidar_distance:
                cv2.putText(frame, f"LiDAR: {data.target_lidar_distance:.2f}m",
                            (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Draw mini LiDAR plot in corner
            plot_size = 150
            plot = np.zeros((plot_size, plot_size, 3), dtype=np.uint8)
            center = plot_size // 2

            if data.lidar_scan:
                for point in data.lidar_scan.points:
                    if 0 < point.distance < 5:  # Max 5m
                        angle_rad = math.radians(point.angle)
                        scale = plot_size / 10  # 5m radius
                        px = int(center + point.distance * scale * math.sin(angle_rad))
                        py = int(center - point.distance * scale * math.cos(angle_rad))
                        if 0 <= px < plot_size and 0 <= py < plot_size:
                            cv2.circle(plot, (px, py), 1, (0, 0, 255), -1)

            # Draw robot position
            cv2.circle(plot, (center, center), 3, (0, 255, 0), -1)

            # Draw camera FOV
            half_fov = bridge.camera_hfov / 2
            for angle in [-half_fov, half_fov]:
                rad = math.radians(angle)
                ex = int(center + 70 * math.sin(rad))
                ey = int(center - 70 * math.cos(rad))
                cv2.line(plot, (center, center), (ex, ey), (255, 255, 0), 1)

            # Overlay plot on frame
            frame[10:10+plot_size, w-plot_size-10:w-10] = plot

            # Info text
            cv2.putText(frame, f"Detections: {len(data.detections)}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(frame, f"Obstacles in FOV: {len(data.obstacles_in_view)}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            cv2.imshow("Sensor Fusion", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


# =============================================================================
# Standalone Test Runner
# =============================================================================

def run_standalone_tests():
    """Run tests without pytest."""
    logger = DebugLogger("IntegrationTest", LogLevel.DEBUG)
    checker = HardwareChecker(logger)

    logger.header("LiDAR + Camera Integration Tests")

    # Check hardware
    logger.subheader("Hardware Check")
    checker.print_status()

    if not checker.lidar_available or not checker.camera_available:
        logger.error("Both LiDAR and Camera required for integration tests")
        return False

    # Create bridge
    bridge = SensorFusionBridge(
        checker.lidar_device,
        checker.camera_device,
        logger
    )

    # Connect
    logger.subheader("Connection Test")
    success, message = bridge.connect()
    if not success:
        logger.error(f"Connection failed: {message}")
        return False

    # Fused capture
    logger.subheader("Fused Capture Test")
    data = bridge.capture_fused_data()
    if data:
        logger.success("Fused capture OK")
        logger.data("Detections", len(data.detections))
        logger.data("Obstacles in view", len(data.obstacles_in_view))
    else:
        logger.error("Fused capture failed")

    # Concurrent test
    logger.subheader("Concurrent Operation Test")
    result = bridge.run_concurrent_test(duration=3.0)
    logger.report(result)

    # Cleanup
    bridge.disconnect()

    logger.header("Tests Complete")
    return True


# =============================================================================
# Main Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="LiDAR + Camera Integration Tests")
    parser.add_argument("--visualize", action="store_true",
                        help="Run with visualization")
    parser.add_argument("--standalone", action="store_true",
                        help="Run standalone tests")
    args = parser.parse_args()

    if args.visualize:
        logger = DebugLogger("SensorFusion")
        checker = HardwareChecker(logger)

        if not checker.lidar_available or not checker.camera_available:
            logger.error("Both LiDAR and Camera required")
            sys.exit(1)

        bridge = SensorFusionBridge(
            checker.lidar_device,
            checker.camera_device,
            logger
        )

        success, _ = bridge.connect()
        if success:
            run_visualization(bridge)
            bridge.disconnect()

    elif args.standalone:
        success = run_standalone_tests()
        sys.exit(0 if success else 1)

    else:
        import pytest
        sys.exit(pytest.main([__file__, "-v", "-s"]))


if __name__ == "__main__":
    main()
