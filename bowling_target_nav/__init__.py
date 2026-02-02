"""
Bowling Target Navigation Package
=================================

A modular vision-based target detection and navigation system for ROS2.

Subpackages:
    - nodes: ROS2 nodes (vision, follower, arduino, gui)
    - detectors: Detection backends (YOLO, binary)
    - hardware: Hardware interfaces (Arduino bridge)
    - utils: Utility modules (distance estimation)

Example:
    from bowling_target_nav.detectors import YoloDetector, Detection
    from bowling_target_nav.utils import DistanceEstimator
    from bowling_target_nav.hardware import ArduinoBridge
"""

__version__ = '1.0.0'
