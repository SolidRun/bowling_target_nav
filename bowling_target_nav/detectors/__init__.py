"""
Detectors Module
================

Provides object detection with multiple backend support:
- YOLO ONNX Runtime (default, CPU fallback)
- V2N DRP-AI Binary (hardware-accelerated on RZ/V2N)
"""

from .base import DetectorBase, Detection, DetectionResult
from .yolo_onnx_detector import YoloOnnxDetector
from .drp_binary_detector import DrpBinaryDetector

__all__ = [
    'DetectorBase',
    'Detection',
    'DetectionResult',
    'YoloOnnxDetector',
    'DrpBinaryDetector',
]
