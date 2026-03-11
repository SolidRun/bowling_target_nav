"""Target selection from detection results.

Provides a simple nearest-target selection strategy.  The detection list
is pre-filtered by class name upstream (in the detection pipeline); this
module only ranks by distance.

Used by the ROS control loop to choose which detection to navigate toward
when multiple objects are visible.
"""

import math


def find_best_target(detections):
    """Select the closest valid target from a list of detections.

    Skips entries with missing, non-finite, or non-positive distances.

    Args:
        detections: List of detection dicts.  Each dict is expected to
            contain at least a ``'distance'`` key (float, meters).
            Additional keys (``'angle'``, ``'bbox_clipped'``, etc.) are
            passed through unchanged.

    Returns:
        The detection dict with the smallest valid distance, or None if
        no valid detection exists.
    """
    best = None
    best_dist = float('inf')
    for det in detections:
        dist = det.get('distance', float('inf'))
        if not isinstance(dist, (int, float)) or not math.isfinite(dist) or dist <= 0:
            continue
        if dist < best_dist:
            best_dist = dist
            best = det
    return best
