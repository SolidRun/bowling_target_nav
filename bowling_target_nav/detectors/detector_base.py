"""
Abstract Base Class for Object Detectors
=========================================

This module defines the interface that all detection backends must implement.
This allows easy swapping between different detectors (YOLO, custom binary, etc.)

Usage:
    class MyDetector(DetectorBase):
        def detect(self, frame):
            # Your detection logic
            return [Detection(...), ...]
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional
import numpy as np


@dataclass
class Detection:
    """
    Represents a single object detection.

    Attributes:
        x1, y1: Top-left corner of bounding box (pixels)
        x2, y2: Bottom-right corner of bounding box (pixels)
        confidence: Detection confidence score (0.0 - 1.0)
        class_id: Integer class identifier
        class_name: Human-readable class name
    """
    x1: int
    y1: int
    x2: int
    y2: int
    confidence: float
    class_id: int
    class_name: str

    @property
    def width(self) -> int:
        """Bounding box width in pixels."""
        return self.x2 - self.x1

    @property
    def height(self) -> int:
        """Bounding box height in pixels."""
        return self.y2 - self.y1

    @property
    def area(self) -> int:
        """Bounding box area in pixels."""
        return self.width * self.height

    @property
    def center_x(self) -> float:
        """X coordinate of bounding box center."""
        return (self.x1 + self.x2) / 2.0

    @property
    def center_y(self) -> float:
        """Y coordinate of bounding box center."""
        return (self.y1 + self.y2) / 2.0

    def to_dict(self) -> dict:
        """Convert to dictionary for serialization."""
        return {
            'x1': self.x1, 'y1': self.y1,
            'x2': self.x2, 'y2': self.y2,
            'confidence': self.confidence,
            'class_id': self.class_id,
            'class_name': self.class_name,
            'width': self.width,
            'height': self.height,
            'center_x': self.center_x,
            'center_y': self.center_y,
        }


class DetectorBase(ABC):
    """
    Abstract base class for all object detectors.

    Subclasses must implement:
        - detect(frame) -> List[Detection]

    Optional overrides:
        - get_class_names() -> List[str]
        - cleanup()
    """

    @abstractmethod
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Run detection on a single frame.

        Args:
            frame: BGR image as numpy array (H, W, 3)

        Returns:
            List of Detection objects found in the frame
        """
        pass

    def get_class_names(self) -> List[str]:
        """
        Get list of class names this detector can identify.

        Returns:
            List of class name strings
        """
        return []

    def filter_by_class(self, detections: List[Detection],
                        target_class: str) -> List[Detection]:
        """
        Filter detections to only include specified class.

        Args:
            detections: List of all detections
            target_class: Class name to filter for

        Returns:
            Filtered list of detections
        """
        return [d for d in detections if d.class_name == target_class]

    def get_largest(self, detections: List[Detection]) -> Optional[Detection]:
        """
        Get the detection with the largest bounding box area.
        Useful for finding the closest/most prominent object.

        Args:
            detections: List of detections

        Returns:
            Detection with largest area, or None if list is empty
        """
        if not detections:
            return None
        return max(detections, key=lambda d: d.area)

    def get_most_confident(self, detections: List[Detection]) -> Optional[Detection]:
        """
        Get the detection with highest confidence score.

        Args:
            detections: List of detections

        Returns:
            Detection with highest confidence, or None if list is empty
        """
        if not detections:
            return None
        return max(detections, key=lambda d: d.confidence)

    def cleanup(self):
        """
        Release any resources held by the detector.
        Called when detector is no longer needed.
        """
        pass
