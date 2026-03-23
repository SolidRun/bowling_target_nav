"""
Hardware Module
===============

Provides hardware abstractions with pluggable real and mock implementations.
Each device has an abstract base class, a real implementation (serial/V4L2),
a mock for desktop testing, and a ``create_*()`` factory function.

Devices:
    Arduino motor controller -- Mecanum 4-motor drive via USB serial (115200 baud).
    Camera capture           -- USB camera via OpenCV VideoCapture (V4L2 backend).
    LiDAR sensor             -- RPLidar 2D scanner via USB serial.

Two Arduino modules coexist for different architectural patterns:

    arduino.py        Blocking (synchronous) serial model. Each send_command()
                      call writes a command and waits for the response before
                      returning. Includes mecanum inverse kinematics math for
                      converting m/s to PWM. Has a MockArduino for testing.
                      Classes: ArduinoBase, ArduinoBridge, MockArduino.
                      Used by: app/odometry_node.py, unit tests.

    arduino_bridge.py Non-blocking (asynchronous) serial bridge. Background
                      daemon threads handle reading and reconnection so the
                      caller never blocks on serial I/O. Incoming telemetry
                      and responses are dispatched via an on_response callback.
                      Classes: ArduinoBridge (aliased as ArduinoBridgeDriver),
                               ArduinoConfig, ArduinoState.
                      Used by: app/arduino_node.py (ROS2 production driver).

Firmware protocol: Plain text, newline-terminated, 115200 baud, 200ms VEL
watchdog. See individual module docstrings for full command reference.
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
