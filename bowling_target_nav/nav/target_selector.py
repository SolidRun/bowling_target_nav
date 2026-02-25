"""Target selection from detection results."""


def find_best_target(detections):
    """Find the closest bowling pin from a list of detections.

    Args:
        detections: List of detection dicts with 'distance' key.

    Returns:
        Best detection dict or None.
    """
    best = None
    best_dist = float('inf')
    for det in detections:
        dist = det.get('distance', float('inf'))
        if dist < best_dist:
            best_dist = dist
            best = det
    return best
