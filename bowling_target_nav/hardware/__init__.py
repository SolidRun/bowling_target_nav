"""
Hardware Module
===============

Provides hardware abstractions with pluggable implementations:
- Arduino motor controller (real and mock)
- Camera capture (real and mock)
- LiDAR sensor (real and mock)

Uses Adapter and Factory patterns for hardware abstraction.

Two Arduino modules coexist for different use cases:
    arduino.py        Kinematics-aware bridge (ArduinoBridge, MockArduino)
                      Used by: odometry_node, tests
    arduino_bridge.py Production serial driver (ArduinoBridge as ArduinoBridgeDriver)
                      Used by: arduino_driver_node
"""

from .arduino import (
    ArduinoBase,
    ArduinoBridge,
    MockArduino,
    EncoderData,
    create_arduino,
    WHEEL_RADIUS_M,
    WHEELBASE_M,
    TRACK_WIDTH_M,
    ENCODER_CPR,
)
from .arduino_bridge import (
    ArduinoBridge as ArduinoBridgeDriver,
    ArduinoConfig,
    ArduinoState,
)
from .camera import CameraBase, CameraCapture, MockCamera, create_camera
from .lidar import LidarBase, LidarBridge, MockLidar, create_lidar

__all__ = [
    # Arduino (kinematics-aware, arduino.py)
    'ArduinoBase',
    'ArduinoBridge',
    'MockArduino',
    'EncoderData',
    'create_arduino',
    'WHEEL_RADIUS_M',
    'WHEELBASE_M',
    'TRACK_WIDTH_M',
    'ENCODER_CPR',

    # Arduino (production serial driver, arduino_bridge.py)
    'ArduinoBridgeDriver',
    'ArduinoConfig',
    'ArduinoState',

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
