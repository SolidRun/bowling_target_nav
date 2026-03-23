"""Lightweight detection data container used throughout the pipeline.

Provides the Det dataclass, a slots-based container that replaces raw dicts
for passing detection results between the camera worker, detection filters,
navigator, and GUI panels. Using a dataclass instead of dicts ensures
misspelled field names raise AttributeError immediately.

Used by: camera_worker.py, detection_filters.py, nav_controller.py,
navigator.py, and all GUI panels that display detection info.
"""

from __future__ import annotations
from dataclasses import dataclass


@dataclass(slots=True)
class Det:
    """A single detection result flowing through the pipeline.

    This replaces raw dicts -- any misspelled field name causes an
    immediate AttributeError instead of a silent KeyError default.
    """
    class_name: str           # Detected class name (e.g., "bowling-pin").
    confidence: float          # Detection confidence in [0.0, 1.0].
    bbox: tuple                # Bounding box as (x1, y1, x2, y2) in pixels.
    distance: float = 0.0     # Estimated distance to object in meters.
    angle: float = 0.0        # Horizontal angle from camera center in radians.
    bbox_clipped: bool = False # True if bbox touches the frame edge.

    @property
    def width(self) -> int:
        """Bounding box width in pixels."""
        return self.bbox[2] - self.bbox[0]

    @property
    def height(self) -> int:
        """Bounding box height in pixels."""
        return self.bbox[3] - self.bbox[1]

    @property
    def center(self) -> tuple:
        """Bounding box center as (cx, cy) in pixels."""
        return ((self.bbox[0] + self.bbox[2]) / 2.0,
                (self.bbox[1] + self.bbox[3]) / 2.0)

