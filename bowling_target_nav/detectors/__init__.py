"""
Detectors Module
================

Provides pluggable object detection with multiple backend support:
- YOLO ONNX (default)
- V2N DRP Binary application
- Mock detector (for testing)

Uses Strategy pattern for easy swapping between implementations.
"""

from .base import DetectorBase, Detection, DetectionResult
from .yolo_onnx_detector import YoloOnnxDetector
from .drp_binary_detector import DrpBinaryDetector
from .mock_detector import MockDetector
from .factory import DetectorFactory, create_detector

__all__ = [
    'DetectorBase',
    'Detection',
    'DetectionResult',
    'YoloOnnxDetector',
    'DrpBinaryDetector',
    'MockDetector',
    'DetectorFactory',
    'create_detector',
]
