"""
Target Navigation Package
=========================

Autonomous target detection and navigation for the RZ/V2N robot.
Uses DRP-AI YOLO detection via a C++ binary,
and mecanum holonomic drive with Arduino motor controller firmware.

Architecture:
    The GUI process launches ROS2 nodes (NavNode, CameraNode, ArduinoNode,
    OdometryNode) as child processes. All inter-process communication uses
    ROS2 topics. The GUI subscribes to state updates via a lightweight
    bridge node. ``config.py`` is the single source of truth for all
    hardware constants and default parameters.

Subpackages:
    - app:        Application entry points and ROS2 node definitions.
    - nav:        Navigation algorithms (holonomic drive, obstacle avoidance,
                  blind approach, spiral search).
    - detectors:  Detection backends (DRP-AI hardware C++ binary).
    - hardware:   Hardware abstractions (Arduino, camera, LiDAR).
    - gui:        GTK3 interface (main window, settings, map/camera panels).
    - state:      Thread-safe shared state (sensor, settings, nav stores).
    - utils:      Utilities (distance estimation, logging).

Example:
    from target_nav.state import state
    from target_nav.detectors import DrpBinaryDetector, Detection
    from target_nav.utils import DistanceEstimator
"""

__version__ = '1.1.0'
