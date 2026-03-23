"""Abstract base class for all object detectors (Strategy pattern).

Defines three core types:

  - **Detection**: A single detected object with bbox, confidence, class,
    and optional distance estimate. This is the "rich" detection type used
    internally by DetectorBase and the DistanceEstimator. The lightweight
    Det dataclass (in detectors/detection.py) is the type that flows through
    the rest of the pipeline (navigator, GUI, ROS2 topics).

  - **DetectionResult**: Container for a batch of Detection objects from one
    inference pass, with timing, class filtering, and error reporting.

  - **DetectorBase**: Abstract base that concrete backends (DRP-AI pipe,
    DRP-AI stream, etc.) must subclass. Provides initialize/detect/shutdown
    lifecycle and inference statistics.

Subclasses implement ``_load_model()`` and ``_detect_impl(frame)``. The public
``detect()`` method handles timing and error wrapping.

This module is used by the camera worker (app/camera_worker.py) and the
detection filtering pipeline (app/detection_filters.py).
"""

import logging
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np

from target_nav.config import TARGET_CLASS_NAME

logger = logging.getLogger(__name__)


@dataclass
class Detection:
    """
    Represents a single object detection.

    Attributes:
        class_name: Name of detected class (e.g., "bowling-pin")
        class_id: Numeric class ID
        confidence: Detection confidence (0.0 - 1.0)
        bbox: Bounding box as (x1, y1, x2, y2) in pixels
        center: Center point (cx, cy) in pixels
        area: Bounding box area in pixels
        distance: Estimated distance in meters (if available)
    """
    class_name: str
    class_id: int
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    center: Tuple[int, int] = field(default=(0, 0))
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

    def filter_by_class(self, class_name: str) -> List[Detection]:
        """Filter detections by class name."""
        return [d for d in self.detections if d.class_name == class_name]


class DetectorBase(ABC):
    """Abstract base class for object detectors (Strategy pattern).

    All detector implementations must inherit from this class and implement
    the abstract methods. This allows easy swapping between different
    detection backends (DRP-AI pipe, DRP-AI stream, etc.).

    Interface contract for subclasses:
      - ``_load_model()``: Called once by ``initialize()``. Must load/start
        the detection backend (model file, subprocess, etc.). Raise on
        failure -- ``initialize()`` catches and returns False.
      - ``_detect_impl(frame)``: Called by ``detect()`` for each frame.
        Must return a ``DetectionResult``. Should NOT handle timing --
        the base class wraps it with timing and error handling.
      - ``_cleanup()``: Optional. Called by ``shutdown()`` to release
        resources. Default is a no-op.
      - ``name`` (property): Human-readable detector name for logging.
      - ``supported_classes`` (property): List of class names this
        detector can recognize.

    Lifecycle:
      1. ``__init__()`` -- store config, no heavy I/O.
      2. ``initialize()`` -- calls ``_load_model()``, sets ``_initialized``.
      3. ``detect(frame)`` -- repeated calls, returns ``DetectionResult``.
      4. ``shutdown()`` -- calls ``_cleanup()``, clears ``_initialized``.

    Thread safety: ``detect()`` is called from the camera worker thread.
    Subclasses must ensure their ``_detect_impl`` is safe for the calling
    pattern (typically single-threaded from one camera worker).

    Usage::

        class MyDetector(DetectorBase):
            def _load_model(self):
                ...
            def _detect_impl(self, frame):
                return DetectionResult(...)

        detector = MyDetector(confidence_threshold=0.5)
        detector.initialize()
        result = detector.detect(frame)
    """

    def __init__(
        self,
        confidence_threshold: float = 0.5,
        target_class: str = TARGET_CLASS_NAME,
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
        """Load the detection model or start the detection backend.

        Called once by ``initialize()``. Implementations should perform all
        heavy setup here (loading model files, launching subprocesses, etc.).

        Raises:
            Any exception -- ``initialize()`` catches it, logs, and returns False.
        """
        pass

    def _cleanup(self) -> None:
        """Release backend resources (model handles, subprocesses, etc.).

        Override in subclasses that allocate resources in ``_load_model()``.
        Called by ``shutdown()``. Default implementation is a no-op.
        """
        pass

    @abstractmethod
    def _detect_impl(self, frame: np.ndarray) -> DetectionResult:
        """Run inference on a single frame. Must be implemented by subclasses.

        The base class ``detect()`` method handles timing, statistics, and
        error wrapping, so this method should focus only on inference.

        Args:
            frame: Input image as numpy array (BGR, HxWx3, uint8).

        Returns:
            DetectionResult with detections list populated. Set
            ``success=False`` and ``error=...`` on backend failure
            instead of raising.
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

