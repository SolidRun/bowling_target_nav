"""Distance and angle estimation from bounding-box geometry.

Converts 2D bounding-box information to estimated 3D position using the
pinhole camera model.  The core assumption is that the target has a known
real-world height, so bounding-box height in pixels is inversely
proportional to distance.  Horizontal angle is computed from the pixel
offset of the box center relative to the frame center.

Math:
    Distance (pinhole inverse proportion):
        d = ref_distance * (ref_box_height / current_box_height)

    Focal length (precomputed from horizontal FOV):
        f_px = (image_width / 2) / tan(hfov / 2)

    Horizontal angle (positive = right of center):
        angle = atan2(center_x - image_width/2, f_px)

    Robot-frame position (ROS convention: X forward, Y left):
        x = d * cos(angle)
        y = -d * sin(angle)   (negative because camera-right = robot -Y)

Architecture:
    Instantiated by the camera worker with calibration values from
    SettingsStore.  Called on every accepted detection to fill in the
    ``distance`` and ``angle`` fields of the Det object.  Also used by
    Navigator for target bearing computation.

Key classes:
    DistanceEstimator      -- continuous distance/angle estimation.

Calibration workflow:
    1. Place target at a known distance (e.g. 1.0 m).
    2. Detect it and record bounding-box height in pixels (ref_box_height).
    3. Set ``reference_box_height`` and ``reference_distance`` via the
       constructor, or call ``estimator.calibrate(detection, known_distance)``
       at runtime.
    4. The GUI Settings > Sensors > Calibration tab automates steps 1-3
       (see setup_tab.py ``_on_calibrate``).

Thread safety:
    Instances are not shared across threads; each thread/process creates
    its own.  No internal locking.

Related modules:
    gui/settings_tabs/setup_tab.py -- GUI calibration workflow.
    state/settings_store.py        -- stores ref_box_height + ref_distance.
"""

import math
from typing import Tuple

from target_nav.detectors import Detection
from target_nav.config import (
    DEFAULT_REF_BOX_HEIGHT, DEFAULT_REF_DISTANCE,
    DEFAULT_FRAME_W, DEFAULT_FRAME_H, DEFAULT_CAMERA_FOV,
)
from target_nav.utils.log import get_logger

logger = get_logger('utils.distance_estimator')


class DistanceEstimator:
    """Estimates distance and angle to a detected object from bounding-box size.

    Uses the inverse-proportion relationship between apparent size and
    distance: ``d = ref_distance * (ref_box_height / box_height)``.
    Horizontal angle uses ``atan2(pixel_offset, focal_length)``.

    Args:
        reference_box_height: Bounding-box height (pixels) at reference distance.
        reference_distance: Known distance (meters) for the reference measurement.
        frame_width: Camera frame width in pixels.
        frame_height: Camera frame height in pixels.
        horizontal_fov: Camera horizontal field of view in degrees.

    Raises:
        ValueError: If frame dimensions are non-positive or FOV is out of
            the (0, 180) range.
    """

    def __init__(self,
                 reference_box_height: float = DEFAULT_REF_BOX_HEIGHT,
                 reference_distance: float = DEFAULT_REF_DISTANCE,
                 frame_width: int = DEFAULT_FRAME_W,
                 frame_height: int = DEFAULT_FRAME_H,
                 horizontal_fov: float = DEFAULT_CAMERA_FOV,
                 flip_horizontal: bool = False):
        """Initialize estimator and precompute focal length.

        Args:
            reference_box_height: Box height in pixels at reference distance.
            reference_distance: Distance in meters for the reference measurement.
            frame_width: Camera frame width in pixels.
            frame_height: Camera frame height in pixels.
            horizontal_fov: Horizontal FOV in degrees.
        """
        if frame_width <= 0 or frame_height <= 0:
            raise ValueError(f"Invalid frame dimensions: {frame_width}x{frame_height}")
        if horizontal_fov <= 0 or horizontal_fov >= 180:
            raise ValueError(f"Invalid FOV: {horizontal_fov}")

        self.reference_box_height = reference_box_height
        self.reference_distance = reference_distance
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.horizontal_fov = horizontal_fov
        self.flip_horizontal = flip_horizontal
        # Precompute constants used by estimate() and get_normalized_position()
        self._fov_rad = math.radians(horizontal_fov)
        self._half_width = frame_width / 2.0
        self._half_height = frame_height / 2.0

        # Horizontal focal length in pixels, derived from the pinhole model:
        #   focal_px = (image_width / 2) / tan(hfov / 2)
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
        center_x = detection.center[0]
        offset_x = center_x - self._half_width
        if self.flip_horizontal:       
            offset_x = -offset_x     
        angle = math.atan2(offset_x, self._focal_length)
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
        norm_x = (detection.center[0] - self._half_width) / self._half_width
        norm_y = (detection.center[1] - self._half_height) / self._half_height
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
        logger.info("Calibrated: box_height=%spx at distance=%sm",
                     self.reference_box_height, self.reference_distance)


