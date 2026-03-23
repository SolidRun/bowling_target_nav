"""Utility modules for the target navigation system.

Provides distance/angle estimation from bounding-box geometry and
centralized logging configuration.

Modules:
    distance_estimator -- pinhole-model distance and angle estimation.
    log                -- ``get_logger()`` and ``setup_logging()``.
"""

from .distance_estimator import DistanceEstimator

__all__ = [
    'DistanceEstimator',
]
