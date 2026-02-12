"""
Detector Tests
==============

Unit tests for detector implementations.
"""

import os
import pytest
import numpy as np

import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from bowling_target_nav.detectors.base import Detection, DetectionResult
from bowling_target_nav.detectors.mock_detector import MockDetector
from bowling_target_nav.detectors.factory import DetectorFactory, create_detector


class TestDetection:
    """Test Detection dataclass."""

    def test_detection_creation(self):
        """Test creating a Detection object."""
        det = Detection(
            class_name="bowling_pin",
            class_id=0,
            confidence=0.95,
            bbox=(100, 100, 200, 300)
        )

        assert det.class_name == "bowling_pin"
        assert det.confidence == 0.95
        assert det.bbox == (100, 100, 200, 300)

    def test_detection_center(self):
        """Test center calculation."""
        det = Detection(
            class_name="bowling_pin",
            class_id=0,
            confidence=0.9,
            bbox=(100, 100, 200, 300)
        )

        assert det.center == (150, 200)

    def test_detection_dimensions(self):
        """Test width and height properties."""
        det = Detection(
            class_name="bowling_pin",
            class_id=0,
            confidence=0.9,
            bbox=(100, 100, 200, 350)
        )

        assert det.width == 100
        assert det.height == 250

    def test_detection_area(self):
        """Test area calculation."""
        det = Detection(
            class_name="bowling_pin",
            class_id=0,
            confidence=0.9,
            bbox=(0, 0, 100, 100)
        )

        assert det.area == 10000

    def test_normalize_center(self):
        """Test center normalization."""
        det = Detection(
            class_name="bowling_pin",
            class_id=0,
            confidence=0.9,
            bbox=(300, 220, 340, 260)  # Center at (320, 240)
        )

        det.normalize_center(640, 480)

        # Center should be at (0, 0) since it's in the middle
        assert abs(det.center_normalized[0]) < 0.1
        assert abs(det.center_normalized[1]) < 0.1


class TestDetectionResult:
    """Test DetectionResult dataclass."""

    def test_empty_result(self):
        """Test empty detection result."""
        result = DetectionResult()

        assert result.has_detections == False
        assert result.best_detection is None

    def test_result_with_detections(self):
        """Test result with detections."""
        detections = [
            Detection("bowling_pin", 0, 0.8, (100, 100, 200, 300)),
            Detection("bowling_pin", 0, 0.95, (300, 100, 400, 300)),
        ]
        result = DetectionResult(detections=detections)

        assert result.has_detections == True
        assert result.best_detection.confidence == 0.95

    def test_filter_by_class(self):
        """Test filtering by class name."""
        detections = [
            Detection("bowling_pin", 0, 0.9, (100, 100, 200, 300)),
            Detection("person", 1, 0.85, (300, 100, 400, 300)),
            Detection("bowling_pin", 0, 0.8, (500, 100, 600, 300)),
        ]
        result = DetectionResult(detections=detections)

        pins = result.filter_by_class("bowling_pin")
        assert len(pins) == 2

    def test_filter_by_confidence(self):
        """Test filtering by confidence."""
        detections = [
            Detection("bowling_pin", 0, 0.9, (100, 100, 200, 300)),
            Detection("bowling_pin", 0, 0.6, (300, 100, 400, 300)),
            Detection("bowling_pin", 0, 0.4, (500, 100, 600, 300)),
        ]
        result = DetectionResult(detections=detections)

        high_conf = result.filter_by_confidence(0.7)
        assert len(high_conf) == 1


class TestMockDetector:
    """Test MockDetector implementation."""

    def test_always_detect_mode(self):
        """Test always detect mode."""
        detector = MockDetector(detection_mode="always")
        detector.initialize()

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        result = detector.detect(frame)

        assert result.success
        assert result.has_detections

    def test_never_detect_mode(self):
        """Test never detect mode."""
        detector = MockDetector(detection_mode="never")
        detector.initialize()

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        result = detector.detect(frame)

        assert result.success
        assert not result.has_detections

    def test_random_detect_mode(self):
        """Test random detect mode produces variable results."""
        detector = MockDetector(detection_mode="random", detection_probability=0.5)
        detector.initialize()

        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Run multiple detections
        results = [detector.detect(frame).has_detections for _ in range(100)]

        # Should have both True and False
        assert True in results
        assert False in results

    def test_pattern_detect_mode(self):
        """Test pattern detect mode."""
        detector = MockDetector(
            detection_mode="pattern",
            pattern=[True, True, False]
        )
        detector.initialize()

        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        results = [detector.detect(frame).has_detections for _ in range(6)]

        # Should follow pattern: T, T, F, T, T, F
        assert results == [True, True, False, True, True, False]

    def test_fixed_detections_mode(self):
        """Test fixed detections mode."""
        detector = MockDetector(
            detection_mode="fixed",
            fixed_detections=[
                {"x1": 100, "y1": 100, "x2": 200, "y2": 300, "confidence": 0.95}
            ]
        )
        detector.initialize()

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        result = detector.detect(frame)

        assert result.has_detections
        assert len(result.detections) == 1
        assert result.detections[0].confidence == 0.95

    def test_detector_statistics(self):
        """Test detector statistics tracking."""
        detector = MockDetector(detection_mode="always", inference_delay=0.01)
        detector.initialize()

        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        for _ in range(10):
            detector.detect(frame)

        stats = detector.get_stats()

        assert stats['detection_count'] == 10
        assert stats['average_inference_time'] > 0

    def test_detect_target_method(self):
        """Test detect_target convenience method."""
        detector = MockDetector(
            detection_mode="always",
            target_class="bowling_pin"
        )
        detector.initialize()

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        detection = detector.detect_target(frame)

        assert detection is not None
        assert detection.class_name == "bowling_pin"


class TestDetectorFactory:
    """Test DetectorFactory."""

    def test_create_mock_detector(self):
        """Test creating mock detector."""
        detector = DetectorFactory.create("mock")

        assert detector is not None
        assert "Mock" in detector.name

    def test_create_with_alias(self):
        """Test creating detector with alias."""
        detector = DetectorFactory.create("test")  # alias for mock

        assert detector is not None
        assert "Mock" in detector.name

    def test_available_types(self):
        """Test getting available detector types."""
        types = DetectorFactory.get_available_types()

        assert "yolo_onnx" in types
        assert "drp_binary" in types
        assert "mock" in types

    def test_invalid_type_raises(self):
        """Test invalid detector type raises error."""
        with pytest.raises(ValueError):
            DetectorFactory.create("nonexistent_detector")

    def test_create_detector_function(self):
        """Test create_detector convenience function."""
        detector = create_detector("mock", auto_initialize=True)

        assert detector is not None
        assert detector.is_initialized

    def test_create_and_initialize(self):
        """Test create_and_initialize method."""
        detector = DetectorFactory.create_and_initialize("mock")

        assert detector is not None
        assert detector.is_initialized


class TestYoloOnnxDetector:
    """Test YoloOnnxDetector (without model file)."""

    def test_detector_creation(self):
        """Test detector can be created."""
        from bowling_target_nav.detectors.yolo_onnx_detector import YoloOnnxDetector

        detector = YoloOnnxDetector(
            model_path="nonexistent.onnx",
            confidence_threshold=0.5
        )

        assert detector is not None
        assert detector.name == "YOLO ONNX Detector"

    def test_supported_classes(self):
        """Test supported classes."""
        from bowling_target_nav.detectors.yolo_onnx_detector import YoloOnnxDetector

        detector = YoloOnnxDetector(
            class_names=["bowling_pin", "bottle"]
        )

        assert "bowling_pin" in detector.supported_classes
        assert "bottle" in detector.supported_classes


class TestDrpBinaryDetector:
    """Test DrpBinaryDetector (without binary)."""

    def test_detector_creation(self):
        """Test detector can be created."""
        from bowling_target_nav.detectors.drp_binary_detector import DrpBinaryDetector

        detector = DrpBinaryDetector(
            binary_path="/nonexistent/binary",
            confidence_threshold=0.5
        )

        assert detector is not None
        assert detector.name == "DRP Binary Detector"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
