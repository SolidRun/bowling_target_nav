"""Object detection backends for V2N DRP-AI hardware accelerator.

Exports the base class (DetectorBase), the lightweight Det container, the
internal Detection type (used by DetectorBase and DistanceEstimator),
DetectionResult (batch container), and the DRP-AI pipe detector
(DrpBinaryDetector). The stream detector (DrpStreamDetector) is also in
drp_binary_detector.py and imported directly where needed (e.g.,
app/camera_worker.py).
"""

from .base import DetectorBase, Detection, DetectionResult
from .detection import Det
from .drp_binary_detector import DrpBinaryDetector

__all__ = [
    'DetectorBase',
    'Det',
    'Detection',
    'DetectionResult',
    'DrpBinaryDetector',
]
