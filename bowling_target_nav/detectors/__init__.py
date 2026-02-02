"""
Object Detection Backends for Bowling Target Navigation
"""

from .detector_base import Detection, DetectorBase
from .yolo_detector import YoloDetector
from .bin_detector import BinDetector, BinDetectorStub

__all__ = [
    'Detection',
    'DetectorBase',
    'YoloDetector',
    'BinDetector',
    'BinDetectorStub',
]
