"""
Detector Base Class
===================

Abstract base class for all object detectors.
Implements Strategy pattern for pluggable detection backends.
"""

import logging
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class Detection:
    """
    Represents a single object detection.

    Attributes:
        class_name: Name of detected class (e.g., "bowling_pin")
        class_id: Numeric class ID
        confidence: Detection confidence (0.0 - 1.0)
        bbox: Bounding box as (x1, y1, x2, y2) in pixels
        center: Center point (cx, cy) in pixels
        center_normalized: Center point normalized to (-1, 1) range
        area: Bounding box area in pixels
        distance: Estimated distance in meters (if available)
    """
    class_name: str
    class_id: int
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    center: Tuple[int, int] = field(default=(0, 0))
    center_normalized: Tuple[float, float] = field(default=(0.0, 0.0))
    area: int = 0
    distance: Optional[float] = None

    def __post_init__(self):
        """Calculate derived values."""
        x1, y1, x2, y2 = self.bbox
        self.center = ((x1 + x2) // 2, (y1 + y2) // 2)
        self.area = (x2 - x1) * (y2 - y1)

    @property
    def width(self) -> int:
        """Bounding box width."""
        return self.bbox[2] - self.bbox[0]

    @property
    def height(self) -> int:
        """Bounding box height."""
        return self.bbox[3] - self.bbox[1]

    @property
    def center_x(self) -> int:
        """Center X coordinate."""
        return self.center[0]

    @property
    def center_y(self) -> int:
        """Center Y coordinate."""
        return self.center[1]

    def normalize_center(self, frame_width: int, frame_height: int) -> None:
        """Normalize center coordinates to -1 to 1 range."""
        cx, cy = self.center
        nx = (cx - frame_width / 2) / (frame_width / 2)
        ny = (cy - frame_height / 2) / (frame_height / 2)
        self.center_normalized = (nx, ny)


@dataclass
class DetectionResult:
    """
    Result of a detection operation.

    Attributes:
        detections: List of detected objects
        inference_time: Time taken for inference in seconds
        timestamp: Unix timestamp of detection
        frame_size: Size of input frame (width, height)
        success: Whether detection was successful
        error: Error message if detection failed
    """
    detections: List[Detection] = field(default_factory=list)
    inference_time: float = 0.0
    timestamp: float = field(default_factory=time.time)
    frame_size: Tuple[int, int] = (0, 0)
    success: bool = True
    error: str = ""

    @property
    def has_detections(self) -> bool:
        """Check if any objects were detected."""
        return len(self.detections) > 0

    @property
    def best_detection(self) -> Optional[Detection]:
        """Get detection with highest confidence."""
        if not self.detections:
            return None
        return max(self.detections, key=lambda d: d.confidence)

    def filter_by_class(self, class_name: str) -> List[Detection]:
        """Filter detections by class name."""
        return [d for d in self.detections if d.class_name == class_name]

    def filter_by_confidence(self, min_confidence: float) -> List[Detection]:
        """Filter detections by minimum confidence."""
        return [d for d in self.detections if d.confidence >= min_confidence]


class DetectorBase(ABC):
    """
    Abstract base class for object detectors.

    All detector implementations must inherit from this class and implement
    the abstract methods. This allows easy swapping between different
    detection backends (YOLO ONNX, DRP binary, etc.)

    Usage:
        class MyDetector(DetectorBase):
            def _load_model(self):
                # Load your model
                pass

            def _detect_impl(self, frame):
                # Run detection
                return DetectionResult(...)

        detector = MyDetector(config)
        detector.initialize()
        result = detector.detect(frame)
    """

    def __init__(
        self,
        confidence_threshold: float = 0.5,
        target_class: str = "bowling_pin",
        **kwargs
    ):
        """
        Initialize detector.

        Args:
            confidence_threshold: Minimum confidence for detections
            target_class: Primary target class to detect
            **kwargs: Additional detector-specific parameters
        """
        self.confidence_threshold = confidence_threshold
        self.target_class = target_class
        self._initialized = False
        self._model = None

        # Statistics
        self._detection_count = 0
        self._total_inference_time = 0.0
        self._last_result: Optional[DetectionResult] = None

    @property
    def is_initialized(self) -> bool:
        """Check if detector is initialized."""
        return self._initialized

    @property
    def average_inference_time(self) -> float:
        """Get average inference time in seconds."""
        if self._detection_count == 0:
            return 0.0
        return self._total_inference_time / self._detection_count

    @property
    @abstractmethod
    def name(self) -> str:
        """Get detector name."""
        pass

    @property
    @abstractmethod
    def supported_classes(self) -> List[str]:
        """Get list of supported class names."""
        pass

    def initialize(self) -> bool:
        """
        Initialize the detector (load model, etc.)

        Returns:
            True if initialization successful, False otherwise
        """
        if self._initialized:
            logger.warning(f"{self.name} already initialized")
            return True

        try:
            logger.info(f"Initializing {self.name}...")
            self._load_model()
            self._initialized = True
            logger.info(f"{self.name} initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize {self.name}: {e}")
            return False

    def shutdown(self) -> None:
        """Shutdown the detector and release resources."""
        if not self._initialized:
            return

        try:
            logger.info(f"Shutting down {self.name}...")
            self._cleanup()
            self._initialized = False
            self._model = None
            logger.info(f"{self.name} shut down")
        except Exception as e:
            logger.error(f"Error shutting down {self.name}: {e}")

    @abstractmethod
    def _load_model(self) -> None:
        """
        Load the detection model.
        Must be implemented by subclasses.
        """
        pass

    def _cleanup(self) -> None:
        """
        Cleanup resources. Override in subclasses if needed.
        """
        pass

    @abstractmethod
    def _detect_impl(self, frame: np.ndarray) -> DetectionResult:
        """
        Perform detection on a frame.
        Must be implemented by subclasses.

        Args:
            frame: Input image as numpy array (BGR format)

        Returns:
            DetectionResult with detected objects
        """
        pass

    def detect(self, frame: np.ndarray) -> DetectionResult:
        """
        Detect objects in a frame.

        Args:
            frame: Input image as numpy array (BGR format)

        Returns:
            DetectionResult with detected objects
        """
        if not self._initialized:
            return DetectionResult(
                success=False,
                error=f"{self.name} not initialized"
            )

        if frame is None or frame.size == 0:
            return DetectionResult(
                success=False,
                error="Invalid frame"
            )

        try:
            start_time = time.time()
            result = self._detect_impl(frame)
            result.inference_time = time.time() - start_time
            result.frame_size = (frame.shape[1], frame.shape[0])

            # Normalize detection centers
            for det in result.detections:
                det.normalize_center(frame.shape[1], frame.shape[0])

            # Update statistics
            self._detection_count += 1
            self._total_inference_time += result.inference_time
            self._last_result = result

            return result

        except Exception as e:
            logger.error(f"Detection error: {e}")
            return DetectionResult(
                success=False,
                error=str(e)
            )

    def detect_target(self, frame: np.ndarray) -> Optional[Detection]:
        """
        Detect the primary target in a frame.

        Args:
            frame: Input image as numpy array (BGR format)

        Returns:
            Best detection of target class, or None if not found
        """
        result = self.detect(frame)
        if not result.success:
            return None

        # Filter by target class
        targets = result.filter_by_class(self.target_class)
        if not targets:
            return None

        # Return highest confidence detection
        return max(targets, key=lambda d: d.confidence)

    def get_stats(self) -> dict:
        """Get detector statistics."""
        return {
            'name': self.name,
            'initialized': self._initialized,
            'detection_count': self._detection_count,
            'average_inference_time': self.average_inference_time,
            'confidence_threshold': self.confidence_threshold,
            'target_class': self.target_class,
        }
