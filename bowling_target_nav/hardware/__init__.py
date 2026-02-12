"""
Hardware Module
===============

Provides hardware abstractions with pluggable implementations:
- Arduino motor controller (real and mock)
- Camera capture (real and mock)
- LiDAR sensor (real and mock)

Uses Adapter and Factory patterns for hardware abstraction.
"""

from .arduino import ArduinoBase, ArduinoBridge, MockArduino, create_arduino
from .arduino_bridge import (
    ArduinoBridge as ArduinoBridgeDriver,
    ArduinoConfig,
    ArduinoState,
)
from .camera import CameraBase, CameraCapture, MockCamera, create_camera
from .lidar import LidarBase, LidarBridge, MockLidar, create_lidar

__all__ = [
    # Arduino
    'ArduinoBase',
    'ArduinoBridge',
    'ArduinoBridgeDriver',
    'ArduinoConfig',
    'ArduinoState',
    'MockArduino',
    'create_arduino',

    # Camera
    'CameraBase',
    'CameraCapture',
    'MockCamera',
    'create_camera',

    # LiDAR
    'LidarBase',
    'LidarBridge',
    'MockLidar',
    'create_lidar',
]
