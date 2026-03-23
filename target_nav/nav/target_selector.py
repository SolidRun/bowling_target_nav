"""Target selection from detection results.

Uses weighted scoring to pick the best target, balancing:
- Distance (closer is better)
- Confidence (higher DRP-AI confidence is better)
- Frame position (center-frame targets are easier to navigate to)

This rejects false positives that are close but low-confidence, and
prefers high-confidence targets even if slightly farther away.

Used by NavController.step() to choose which Det object to navigate toward.
"""

import math

from target_nav.config import DEFAULT_FRAME_W


def find_best_target(detections):
    """Select the best target using weighted scoring.

    Score = distance * (1.1 - confidence) * center_penalty
    Lower score = better target.

    A high-confidence target at 1.5m beats a low-confidence chair at 1.0m
    because the chair's low confidence inflates its score.

    Args:
        detections: List of Det objects with distance, confidence, bbox.

    Returns:
        The Det with the best (lowest) score, or None.
    """
    best = None
    best_score = float('inf')
    half_w = DEFAULT_FRAME_W / 2.0

    for det in detections:
        dist = det.distance
        if not isinstance(dist, (int, float)) or not math.isfinite(dist) or dist <= 0:
            continue

        conf = max(0.1, det.confidence)

        # Center bias: targets near frame center are easier to navigate to.
        # Edge targets get a mild penalty (1.0 at center, 1.3 at edge).
        cx = (det.bbox[0] + det.bbox[2]) / 2.0
        center_offset = abs(cx - half_w) / half_w  # 0 at center, 1 at edge
        center_penalty = 1.0 + center_offset * 0.3

        # Score: closer + higher confidence + more centered = lower score
        score = dist * (1.1 - conf) * center_penalty

        if score < best_score:
            best_score = score
            best = det

    return best
