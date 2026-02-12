"""
Mock Detector
=============

Mock detector for testing without real hardware or models.
Generates configurable fake detections for development and testing.
"""

import logging
import random
import time
from typing import List, Optional
import numpy as np

from .base import DetectorBase, Detection, DetectionResult

logger = logging.getLogger(__name__)


class MockDetector(DetectorBase):
    """
    Mock detector for testing.

    Generates fake detections based on configuration. Useful for:
    - Unit testing without model files
    - Integration testing without camera
    - UI development and debugging
    - CI/CD pipelines

    Usage:
        # Always detect
        detector = MockDetector(detection_mode="always")

        # Random detections
        detector = MockDetector(detection_mode="random", detection_probability=0.5)

        # Never detect
        detector = MockDetector(detection_mode="never")

        # Pattern-based (alternating)
        detector = MockDetector(detection_mode="pattern", pattern=[True, True, False])

        # Fixed detections
        detector = MockDetector(
            detection_mode="fixed",
            fixed_detections=[
                {"x1": 100, "y1": 100, "x2": 200, "y2": 300, "confidence": 0.95}
            ]
        )
    """

    def __init__(
        self,
        detection_mode: str = "random",
        detection_probability: float = 0.7,
        pattern: Optional[List[bool]] = None,
        fixed_detections: Optional[List[dict]] = None,
        inference_delay: float = 0.05,
        confidence_range: tuple = (0.7, 0.99),
        class_names: Optional[List[str]] = None,
        target_class: str = "bowling_pin",
        **kwargs
    ):
        """
        Initialize mock detector.

        Args:
            detection_mode: "always" | "never" | "random" | "pattern" | "fixed"
            detection_probability: Probability of detection (for random mode)
            pattern: List of bool for pattern mode (cycles through)
            fixed_detections: List of detection dicts for fixed mode
            inference_delay: Simulated inference time in seconds
            confidence_range: (min, max) confidence for generated detections
            class_names: List of class names
            target_class: Primary target class
        """
        super().__init__(
            target_class=target_class,
            **kwargs
        )

        self.detection_mode = detection_mode
        self.detection_probability = detection_probability
        self.pattern = pattern or [True, True, True, False]
        self.fixed_detections = fixed_detections
        self.inference_delay = inference_delay
        self.confidence_range = confidence_range
        self.class_names = class_names or ["bowling_pin"]

        self._pattern_index = 0
        self._frame_count = 0

    @property
    def name(self) -> str:
        return f"Mock Detector ({self.detection_mode})"

    @property
    def supported_classes(self) -> List[str]:
        return self.class_names

    def _load_model(self) -> None:
        """Mock model loading."""
        logger.info(f"Mock detector initialized in '{self.detection_mode}' mode")

    def _should_detect(self) -> bool:
        """Determine if this frame should have a detection."""
        if self.detection_mode == "always":
            return True
        elif self.detection_mode == "never":
            return False
        elif self.detection_mode == "random":
            return random.random() < self.detection_probability
        elif self.detection_mode == "pattern":
            result = self.pattern[self._pattern_index]
            self._pattern_index = (self._pattern_index + 1) % len(self.pattern)
            return result
        elif self.detection_mode == "fixed":
            return self.fixed_detections is not None and len(self.fixed_detections) > 0
        else:
            return False

    def _generate_detection(self, frame_width: int, frame_height: int) -> Detection:
        """Generate a random detection."""
        # Random position (center-ish)
        cx = frame_width // 2 + random.randint(-100, 100)
        cy = frame_height // 2 + random.randint(-50, 50)

        # Random size
        w = random.randint(50, 150)
        h = random.randint(100, 250)

        x1 = max(0, cx - w // 2)
        y1 = max(0, cy - h // 2)
        x2 = min(frame_width, cx + w // 2)
        y2 = min(frame_height, cy + h // 2)

        confidence = random.uniform(*self.confidence_range)

        return Detection(
            class_name=self.target_class,
            class_id=0,
            confidence=confidence,
            bbox=(x1, y1, x2, y2)
        )

    def _get_fixed_detections(self, frame_width: int, frame_height: int) -> List[Detection]:
        """Get fixed detections from configuration."""
        detections = []

        if not self.fixed_detections:
            return detections

        for det_config in self.fixed_detections:
            detections.append(Detection(
                class_name=det_config.get('class_name', self.target_class),
                class_id=det_config.get('class_id', 0),
                confidence=det_config.get('confidence', 0.9),
                bbox=(
                    det_config.get('x1', 100),
                    det_config.get('y1', 100),
                    det_config.get('x2', 200),
                    det_config.get('y2', 300)
                )
            ))

        return detections

    def _detect_impl(self, frame: np.ndarray) -> DetectionResult:
        """Generate mock detections."""
        self._frame_count += 1

        # Simulate inference time
        if self.inference_delay > 0:
            time.sleep(self.inference_delay)

        h, w = frame.shape[:2]
        detections = []

        if self._should_detect():
            if self.detection_mode == "fixed":
                detections = self._get_fixed_detections(w, h)
            else:
                detections = [self._generate_detection(w, h)]

        return DetectionResult(
            detections=detections,
            success=True
        )

    def set_detection_mode(self, mode: str) -> None:
        """Change detection mode at runtime."""
        self.detection_mode = mode
        logger.info(f"Mock detector mode changed to: {mode}")

    def set_detection_probability(self, probability: float) -> None:
        """Change detection probability at runtime."""
        self.detection_probability = max(0.0, min(1.0, probability))

    def add_fixed_detection(
        self,
        x1: int, y1: int, x2: int, y2: int,
        confidence: float = 0.9,
        class_name: Optional[str] = None
    ) -> None:
        """Add a fixed detection."""
        if self.fixed_detections is None:
            self.fixed_detections = []

        self.fixed_detections.append({
            'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
            'confidence': confidence,
            'class_name': class_name or self.target_class
        })

    def clear_fixed_detections(self) -> None:
        """Clear all fixed detections."""
        self.fixed_detections = []
