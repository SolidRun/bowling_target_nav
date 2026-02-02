"""
Distance and Angle Estimator from Bounding Box
===============================================

Converts 2D bounding box information to estimated 3D position.
Uses the principle that larger bounding boxes = closer objects.

Calibration:
    1. Place target at known distance (e.g., 1.0m)
    2. Measure bounding box height in pixels
    3. Set reference_box_height and reference_distance parameters

Usage:
    estimator = DistanceEstimator(
        reference_box_height=100,  # pixels at 1m
        reference_distance=1.0,    # meters
        frame_width=320,
        horizontal_fov=60.0        # camera FOV in degrees
    )

    distance, angle = estimator.estimate(detection)
"""

import math
from typing import Tuple

from bowling_target_nav.detectors import Detection


class DistanceEstimator:
    """
    Estimates distance and angle to detected object from bounding box size.

    The distance estimation assumes:
        - Object size is relatively constant (e.g., bowling pin)
        - Bounding box height is proportional to apparent size
        - Camera has known field of view

    Args:
        reference_box_height: Bounding box height (pixels) at reference distance
        reference_distance: Known distance (meters) for reference measurement
        frame_width: Camera frame width in pixels
        frame_height: Camera frame height in pixels
        horizontal_fov: Camera horizontal field of view in degrees
    """

    def __init__(self,
                 reference_box_height: float = 100.0,
                 reference_distance: float = 1.0,
                 frame_width: int = 320,
                 frame_height: int = 240,
                 horizontal_fov: float = 60.0):

        self.reference_box_height = reference_box_height
        self.reference_distance = reference_distance
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.horizontal_fov = horizontal_fov

        # Precompute
        self._fov_rad = math.radians(horizontal_fov)
        self._half_width = frame_width / 2.0
        self._half_height = frame_height / 2.0

        # Focal length approximation (pixels)
        self._focal_length = self._half_width / math.tan(self._fov_rad / 2.0)

    def estimate(self, detection: Detection) -> Tuple[float, float]:
        """
        Estimate distance and horizontal angle to detected object.

        Args:
            detection: Detection object with bounding box

        Returns:
            Tuple of (distance_meters, angle_radians)
            - distance: Estimated distance to object
            - angle: Horizontal angle from camera center (+ = right, - = left)
        """
        # Distance from box height (inverse relationship)
        # distance = reference_distance * (reference_height / current_height)
        box_height = detection.height
        if box_height <= 0:
            return float('inf'), 0.0

        distance = self.reference_distance * (self.reference_box_height / box_height)

        # Angle from horizontal position
        # Object center X relative to frame center
        center_x = detection.center_x
        offset_x = center_x - self._half_width

        # Angle = atan(offset / focal_length)
        angle = math.atan2(offset_x, self._focal_length)

        return distance, angle

    def estimate_position(self, detection: Detection) -> Tuple[float, float]:
        """
        Estimate X, Y position in robot frame.

        Assumes camera is forward-facing on robot:
            - X = forward distance
            - Y = lateral offset (+ = left, - = right in ROS convention)

        Args:
            detection: Detection object

        Returns:
            Tuple of (x_meters, y_meters) in robot frame
        """
        distance, angle = self.estimate(detection)

        # Convert polar to cartesian
        # Note: In ROS, X is forward, Y is left
        x = distance * math.cos(angle)
        y = -distance * math.sin(angle)  # Negative because camera right = robot right = -Y

        return x, y

    def get_relative_size(self, detection: Detection) -> float:
        """
        Get relative size compared to reference.

        Values:
            > 1.0 = closer than reference
            = 1.0 = at reference distance
            < 1.0 = farther than reference

        Args:
            detection: Detection object

        Returns:
            Relative size ratio
        """
        if detection.height <= 0:
            return 0.0
        return detection.height / self.reference_box_height

    def get_normalized_position(self, detection: Detection) -> Tuple[float, float]:
        """
        Get normalized position in frame (-1 to +1 range).

        Useful for proportional control without distance estimation.

        Args:
            detection: Detection object

        Returns:
            Tuple of (norm_x, norm_y) where:
            - norm_x: -1 (left edge) to +1 (right edge)
            - norm_y: -1 (top) to +1 (bottom)
        """
        norm_x = (detection.center_x - self._half_width) / self._half_width
        norm_y = (detection.center_y - self._half_height) / self._half_height
        return norm_x, norm_y

    def is_centered(self, detection: Detection, threshold: float = 0.1) -> bool:
        """
        Check if detection is approximately centered in frame.

        Args:
            detection: Detection object
            threshold: Maximum normalized distance from center (0.1 = 10%)

        Returns:
            True if object is within threshold of center
        """
        norm_x, _ = self.get_normalized_position(detection)
        return abs(norm_x) < threshold

    def calibrate(self, detection: Detection, known_distance: float):
        """
        Calibrate reference values from a detection at known distance.

        Place target at known distance, detect it, then call this method.

        Args:
            detection: Detection of target at known distance
            known_distance: Actual distance to target in meters
        """
        self.reference_box_height = float(detection.height)
        self.reference_distance = known_distance
        print(f"Calibrated: box_height={self.reference_box_height}px "
              f"at distance={self.reference_distance}m")


class DistanceEstimatorIndex:
    """
    Simple index-based distance estimation.

    Returns discrete distance zones (0=far, 1=medium, 2=close, 3=very close)
    instead of continuous distance values.

    Useful when precise distance isn't needed, just rough proximity.

    Args:
        thresholds: List of box height thresholds for each zone
                   Default: [50, 100, 150] for 4 zones
        frame_width: Camera frame width for angle calculation
        horizontal_fov: Camera horizontal FOV in degrees
    """

    def __init__(self,
                 thresholds: list = None,
                 frame_width: int = 320,
                 horizontal_fov: float = 60.0):

        self.thresholds = thresholds or [50, 100, 150]  # Far, Medium, Close
        self.frame_width = frame_width
        self.horizontal_fov = horizontal_fov

        self._fov_rad = math.radians(horizontal_fov)
        self._half_width = frame_width / 2.0
        self._focal_length = self._half_width / math.tan(self._fov_rad / 2.0)

    def get_distance_index(self, detection: Detection) -> int:
        """
        Get distance zone index.

        Args:
            detection: Detection object

        Returns:
            Index: 0=far, 1=medium, 2=close, 3=very close (or more if more thresholds)
        """
        height = detection.height
        for i, threshold in enumerate(self.thresholds):
            if height < threshold:
                return i
        return len(self.thresholds)

    def get_angle_index(self, detection: Detection, zones: int = 5) -> int:
        """
        Get horizontal angle zone index.

        Args:
            detection: Detection object
            zones: Number of horizontal zones (default 5: far-left, left, center, right, far-right)

        Returns:
            Index from 0 (far left) to zones-1 (far right)
        """
        norm_x = (detection.center_x - self._half_width) / self._half_width
        # Map -1..+1 to 0..zones-1
        index = int((norm_x + 1.0) / 2.0 * zones)
        return max(0, min(zones - 1, index))

    def get_location_index(self, detection: Detection,
                           distance_zones: int = 4,
                           angle_zones: int = 5) -> Tuple[int, int]:
        """
        Get combined location as (distance_index, angle_index).

        Args:
            detection: Detection object
            distance_zones: Number of distance zones
            angle_zones: Number of angle zones

        Returns:
            Tuple of (distance_index, angle_index)
        """
        dist_idx = self.get_distance_index(detection)
        angle_idx = self.get_angle_index(detection, angle_zones)
        return dist_idx, angle_idx
