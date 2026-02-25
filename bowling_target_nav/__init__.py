"""
Bowling Target Navigation Package
=================================

Autonomous bowling pin detection and navigation for the RZ/V2N robot.
Uses SLAM mapping, YOLO AI detection, and mecanum holonomic drive.

Subpackages:
    - state:     Thread-safe shared state (sensor, detection, nav stores)
    - nav:       Navigation algorithms (holonomic drive, obstacle avoidance, blind approach)
    - threads:   Worker threads (ROS2 node, camera capture)
    - gui:       GTK3 interface (main window, settings, map/camera panels)
    - detectors: Detection backends (YOLO ONNX, DRP-AI hardware)
    - hardware:  Hardware abstractions (Arduino, camera, LiDAR)
    - nodes:     ROS2 entry points (main_gui, arduino_driver, odometry, etc.)
    - utils:     Utilities (distance estimation from bounding box)
    - core:      Legacy infrastructure (config, events, state machine) - retained for reference

Example:
    from bowling_target_nav.state import state
    from bowling_target_nav.detectors import YoloOnnxDetector, Detection
    from bowling_target_nav.utils import DistanceEstimator
"""

__version__ = '1.1.0'
