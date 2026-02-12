#!/usr/bin/env python3
"""
Camera and YOLO Detection Test Suite
=====================================

Comprehensive tests for camera capture and YOLO object detection.
Tests camera connection, image quality, and bowling pin detection.

Tests:
    - Camera connection
    - Frame capture
    - Image quality metrics
    - YOLO model loading
    - Object detection
    - Detection accuracy
    - Performance benchmarks

Usage:
    # Run as pytest
    pytest test/test_camera.py -v -s

    # Run standalone with preview
    python3 test/test_camera.py --preview

    # Run detection demo
    python3 test/test_camera.py --detect
"""

import sys
import os
import time
import argparse
from typing import Optional, List, Tuple
from dataclasses import dataclass

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test.utils.debug_logger import DebugLogger, TestResult, LogLevel
from test.utils.hardware_checker import HardwareChecker


# =============================================================================
# Camera Data Structures
# =============================================================================

@dataclass
class FrameStats:
    """Statistics for captured frame."""
    width: int
    height: int
    channels: int
    brightness: float  # 0-255
    contrast: float
    blur_score: float  # Higher = sharper


@dataclass
class DetectionResult:
    """Single detection result."""
    class_name: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int
    distance: float = 0.0
    angle: float = 0.0


# =============================================================================
# Camera Test Bridge
# =============================================================================

class CameraTestBridge:
    """
    Camera capture bridge for testing with debug output.
    """

    def __init__(self, device: str = "/dev/video0",
                 logger: Optional[DebugLogger] = None):
        self.device = device
        self.logger = logger or DebugLogger("Camera")
        self.cap = None
        self.connected = False
        self.detector = None
        self.estimator = None

    def connect(self, width: int = 640, height: int = 480) -> Tuple[bool, str]:
        """Connect to camera with debug output."""
        import cv2

        self.logger.info(f"Connecting to camera: {self.device}")
        self.logger.debug(f"Requested resolution: {width}x{height}")

        try:
            # Try V4L2 backend first (better for embedded)
            self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)

            if not self.cap.isOpened():
                # Fallback to default backend
                self.cap = cv2.VideoCapture(self.device)

            if not self.cap.isOpened():
                return False, "Could not open camera"

            # Set resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            # Get actual resolution
            actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = self.cap.get(cv2.CAP_PROP_FPS)

            self.logger.data("Actual resolution", f"{actual_w}x{actual_h}")
            self.logger.data("FPS", fps)

            # Test capture
            ret, frame = self.cap.read()
            if not ret:
                return False, "Could not capture test frame"

            self.connected = True
            self.logger.success("Camera connected")
            return True, f"Connected at {actual_w}x{actual_h}"

        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
            return False, str(e)

    def disconnect(self):
        """Disconnect from camera."""
        self.logger.info("Disconnecting camera...")
        if self.cap:
            self.cap.release()
        self.cap = None
        self.connected = False
        self.logger.success("Disconnected")

    def capture_frame(self):
        """Capture single frame."""
        if not self.connected or not self.cap:
            return None

        ret, frame = self.cap.read()
        if not ret:
            self.logger.error("Frame capture failed")
            return None

        return frame

    def get_frame_stats(self, frame) -> FrameStats:
        """Calculate frame statistics."""
        import cv2
        import numpy as np

        height, width = frame.shape[:2]
        channels = frame.shape[2] if len(frame.shape) > 2 else 1

        # Convert to grayscale for analysis
        if channels > 1:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        # Brightness (mean)
        brightness = np.mean(gray)

        # Contrast (std deviation)
        contrast = np.std(gray)

        # Blur score (variance of Laplacian - higher = sharper)
        blur_score = cv2.Laplacian(gray, cv2.CV_64F).var()

        return FrameStats(
            width=width,
            height=height,
            channels=channels,
            brightness=brightness,
            contrast=contrast,
            blur_score=blur_score
        )

    def load_yolo(self, model_path: Optional[str] = None) -> Tuple[bool, str]:
        """Load YOLO model for detection."""
        self.logger.info("Loading YOLO model...")

        # Find model path
        if model_path is None:
            paths = [
                os.path.expanduser('~/ros2_ws/src/bowling_target_nav/models/bowling_yolov5.onnx'),
                os.path.expanduser('~/ros2_ws/install/bowling_target_nav/share/bowling_target_nav/models/bowling_yolov5.onnx'),
            ]
            for p in paths:
                if os.path.exists(p):
                    model_path = p
                    break

        if model_path is None or not os.path.exists(model_path):
            return False, "Model file not found"

        self.logger.data("Model path", model_path)

        try:
            # Try to import from package
            from bowling_target_nav.detectors import YoloDetector
            from bowling_target_nav.utils import DistanceEstimator

            self.detector = YoloDetector(
                model_path=model_path,
                conf_threshold=0.5
            )

            self.estimator = DistanceEstimator(
                reference_box_height=100.0,
                reference_distance=1.0,
                frame_width=640,
                frame_height=480,
                horizontal_fov=60.0
            )

            self.logger.success("YOLO model loaded")
            self.logger.data("Classes", self.detector.get_class_names())
            return True, "Model loaded"

        except ImportError as e:
            self.logger.error(f"Import error: {e}")
            return False, f"Import error: {e}"
        except Exception as e:
            self.logger.error(f"Load failed: {e}")
            return False, str(e)

    def detect(self, frame) -> List[DetectionResult]:
        """Run YOLO detection on frame."""
        if self.detector is None:
            self.logger.error("YOLO not loaded")
            return []

        try:
            detections = self.detector.detect(frame)
            results = []

            for det in detections:
                result = DetectionResult(
                    class_name=det.class_name,
                    confidence=det.confidence,
                    x1=det.x1, y1=det.y1,
                    x2=det.x2, y2=det.y2
                )

                # Estimate distance if estimator available
                if self.estimator:
                    distance, angle = self.estimator.estimate(det)
                    result.distance = distance
                    result.angle = angle

                results.append(result)

            return results

        except Exception as e:
            self.logger.error(f"Detection failed: {e}")
            return []

    def benchmark_detection(self, num_frames: int = 30) -> dict:
        """Benchmark detection performance."""
        if not self.connected or self.detector is None:
            return {"error": "Not ready"}

        self.logger.info(f"Benchmarking detection ({num_frames} frames)...")

        times = []
        detections_count = []

        for i in range(num_frames):
            frame = self.capture_frame()
            if frame is None:
                continue

            start = time.time()
            results = self.detect(frame)
            elapsed = time.time() - start

            times.append(elapsed)
            detections_count.append(len(results))

            if (i + 1) % 10 == 0:
                self.logger.debug(f"Frame {i+1}/{num_frames}")

        if not times:
            return {"error": "No frames captured"}

        import numpy as np
        times = np.array(times)

        benchmark = {
            "frames": len(times),
            "avg_time": np.mean(times),
            "min_time": np.min(times),
            "max_time": np.max(times),
            "fps": 1.0 / np.mean(times),
            "avg_detections": np.mean(detections_count),
        }

        self.logger.data("Average time", f"{benchmark['avg_time']*1000:.1f}ms")
        self.logger.data("FPS", f"{benchmark['fps']:.1f}")
        self.logger.data("Avg detections", f"{benchmark['avg_detections']:.1f}")

        return benchmark


# =============================================================================
# Pytest Test Cases
# =============================================================================

class TestCamera:
    """Pytest test class for camera."""

    @classmethod
    def setup_class(cls):
        """Setup test fixtures."""
        cls.logger = DebugLogger("CameraTest", LogLevel.DEBUG)
        cls.checker = HardwareChecker(cls.logger)
        cls.bridge = None

    @classmethod
    def teardown_class(cls):
        """Cleanup after tests."""
        if cls.bridge:
            cls.bridge.disconnect()

    def test_01_hardware_check(self):
        """Test that camera hardware is available."""
        self.logger.header("Test: Hardware Check")

        status = self.checker.check_camera()
        self.logger.data("Device", status.device)
        self.logger.data("Available", status.available)

        if not status.available:
            import pytest
            pytest.skip("Camera not available")

        self.logger.success("Camera hardware found")
        assert status.available

    def test_02_connection(self):
        """Test camera connection."""
        self.logger.header("Test: Connection")

        if not self.checker.camera_available:
            import pytest
            pytest.skip("Camera not available")

        self.__class__.bridge = CameraTestBridge(
            self.checker.camera_device,
            self.logger
        )

        success, message = self.bridge.connect()
        assert success, f"Connection failed: {message}"
        self.logger.success("Connection test passed")

    def test_03_frame_capture(self):
        """Test frame capture."""
        self.logger.header("Test: Frame Capture")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("Camera not connected")

        frame = self.bridge.capture_frame()
        assert frame is not None, "Frame capture failed"

        stats = self.bridge.get_frame_stats(frame)
        self.logger.data("Resolution", f"{stats.width}x{stats.height}")
        self.logger.data("Brightness", f"{stats.brightness:.1f}")
        self.logger.data("Contrast", f"{stats.contrast:.1f}")
        self.logger.data("Sharpness", f"{stats.blur_score:.1f}")

        self.logger.success("Frame capture test passed")

    def test_04_frame_quality(self):
        """Test frame quality metrics."""
        self.logger.header("Test: Frame Quality")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("Camera not connected")

        frame = self.bridge.capture_frame()
        assert frame is not None

        stats = self.bridge.get_frame_stats(frame)

        # Quality checks
        assert stats.brightness > 20, f"Image too dark: {stats.brightness}"
        assert stats.brightness < 240, f"Image too bright: {stats.brightness}"
        assert stats.blur_score > 10, f"Image too blurry: {stats.blur_score}"

        self.logger.success("Frame quality test passed")

    def test_05_yolo_loading(self):
        """Test YOLO model loading."""
        self.logger.header("Test: YOLO Loading")

        if not self.bridge or not self.bridge.connected:
            import pytest
            pytest.skip("Camera not connected")

        success, message = self.bridge.load_yolo()

        if not success:
            self.logger.warning(f"YOLO not available: {message}")
            import pytest
            pytest.skip("YOLO model not available")

        self.logger.success("YOLO loading test passed")

    def test_06_detection(self):
        """Test object detection."""
        self.logger.header("Test: Detection")

        if not self.bridge or self.bridge.detector is None:
            import pytest
            pytest.skip("YOLO not loaded")

        frame = self.bridge.capture_frame()
        assert frame is not None

        results = self.bridge.detect(frame)
        self.logger.data("Detections", len(results))

        for det in results:
            self.logger.info(f"  {det.class_name}: {det.confidence:.2f} "
                             f"({det.x1},{det.y1})-({det.x2},{det.y2})")

        self.logger.success("Detection test passed")


# =============================================================================
# Camera Preview
# =============================================================================

def run_preview(bridge: CameraTestBridge):
    """Run camera preview window."""
    import cv2

    logger = bridge.logger
    logger.header("Camera Preview")
    logger.info("Press Q to quit")

    cv2.namedWindow("Camera Preview", cv2.WINDOW_NORMAL)

    frame_count = 0
    start_time = time.time()

    try:
        while True:
            frame = bridge.capture_frame()
            if frame is None:
                continue

            frame_count += 1

            # Calculate FPS
            elapsed = time.time() - start_time
            if elapsed > 1.0:
                fps = frame_count / elapsed
                logger.debug(f"FPS: {fps:.1f}")
                frame_count = 0
                start_time = time.time()

            # Show stats on frame
            stats = bridge.get_frame_stats(frame)
            cv2.putText(frame, f"Brightness: {stats.brightness:.0f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Sharpness: {stats.blur_score:.0f}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("Camera Preview", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


def run_detection_demo(bridge: CameraTestBridge):
    """Run detection demo with visualization."""
    import cv2

    logger = bridge.logger
    logger.header("Detection Demo")

    # Load YOLO
    success, message = bridge.load_yolo()
    if not success:
        logger.error(f"Could not load YOLO: {message}")
        return

    logger.info("Press Q to quit")
    cv2.namedWindow("Detection Demo", cv2.WINDOW_NORMAL)

    try:
        while True:
            frame = bridge.capture_frame()
            if frame is None:
                continue

            # Run detection
            start = time.time()
            results = bridge.detect(frame)
            elapsed = time.time() - start

            # Draw detections
            for det in results:
                color = (0, 255, 0) if det.class_name == "Pins" else (0, 165, 255)
                cv2.rectangle(frame, (det.x1, det.y1), (det.x2, det.y2), color, 2)

                label = f"{det.class_name}: {det.confidence:.2f}"
                if det.distance > 0:
                    label += f" ({det.distance:.2f}m)"

                cv2.putText(frame, label, (det.x1, det.y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Show info
            fps = 1.0 / elapsed if elapsed > 0 else 0
            cv2.putText(frame, f"FPS: {fps:.1f} | Detections: {len(results)}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            cv2.imshow("Detection Demo", frame)

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
    logger = DebugLogger("CameraTest", LogLevel.DEBUG)
    checker = HardwareChecker(logger)

    logger.header("Camera and Detection Tests")

    # Check hardware
    logger.subheader("Hardware Check")
    status = checker.check_camera()
    if not status.available:
        logger.error(f"Camera not available: {status.error}")
        return False

    logger.success(f"Camera found on {status.device}")

    # Create bridge
    bridge = CameraTestBridge(status.device, logger)

    # Test connection
    logger.subheader("Connection Test")
    success, message = bridge.connect()
    if not success:
        logger.error(f"Connection failed: {message}")
        return False

    # Test capture
    logger.subheader("Frame Capture Test")
    frame = bridge.capture_frame()
    if frame is not None:
        stats = bridge.get_frame_stats(frame)
        logger.data("Resolution", f"{stats.width}x{stats.height}")
        logger.data("Brightness", f"{stats.brightness:.1f}")
        logger.data("Sharpness", f"{stats.blur_score:.1f}")
        logger.success("Frame capture OK")
    else:
        logger.error("Frame capture failed")

    # Test YOLO
    logger.subheader("YOLO Detection Test")
    success, message = bridge.load_yolo()
    if success:
        frame = bridge.capture_frame()
        if frame is not None:
            results = bridge.detect(frame)
            logger.data("Detections", len(results))
            for det in results:
                logger.info(f"  {det.class_name}: {det.confidence:.2f}")

            # Benchmark
            logger.subheader("Performance Benchmark")
            benchmark = bridge.benchmark_detection(num_frames=20)
            if "error" not in benchmark:
                logger.success(f"Detection at {benchmark['fps']:.1f} FPS")
    else:
        logger.warning(f"YOLO not available: {message}")

    # Cleanup
    bridge.disconnect()

    logger.header("Tests Complete")
    return True


# =============================================================================
# Main Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="Camera and Detection Tests")
    parser.add_argument("--preview", action="store_true",
                        help="Run camera preview")
    parser.add_argument("--detect", action="store_true",
                        help="Run detection demo")
    parser.add_argument("--standalone", action="store_true",
                        help="Run standalone tests")
    parser.add_argument("--device", default="/dev/video0",
                        help="Camera device")
    args = parser.parse_args()

    if args.preview or args.detect:
        logger = DebugLogger("CameraDemo")
        bridge = CameraTestBridge(args.device, logger)
        success, _ = bridge.connect()

        if success:
            if args.detect:
                run_detection_demo(bridge)
            else:
                run_preview(bridge)
            bridge.disconnect()
        else:
            logger.error("Could not connect to camera")
            sys.exit(1)

    elif args.standalone:
        success = run_standalone_tests()
        sys.exit(0 if success else 1)

    else:
        import pytest
        sys.exit(pytest.main([__file__, "-v", "-s"]))


if __name__ == "__main__":
    main()
