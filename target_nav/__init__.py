"""
Target Navigation Package
=========================

Autonomous bowling-pin detection and navigation for the RZ/V2N robot.
Uses DRP-AI YOLO detection via a C++ binary,
and mecanum holonomic drive with Arduino motor controller firmware.

Architecture (3-process, lock-free struct SHM):
    Core 0: GUI (pure GTK -- reads struct SHM directly, no ROS2, no numpy
            in main thread).
    Core 1: Nav process (ROS2 navigation, Kalman tracker, motor commands)
            -- HIGH PRIORITY (nice -5).
    Core 2: Camera Python process (DRP-AI subprocess management, detection
            tracking).
    Core 3: DRP-AI C++ binary (pinned here to isolate heavy CPU from nav).

    Inter-process communication uses lock-free SPSC struct SHM channels
    with torn-read protection via sequence numbers. No locks, no JSON
    serialization for real-time data, no Bridge process.
    ``config.py`` is the single source of truth for all hardware constants
    and default parameters.

Subpackages:
    - app:        Application entry points and ROS2 node definitions.
    - nav:        Navigation algorithms (holonomic drive, obstacle avoidance,
                  blind approach, spiral search).
    - detectors:  Detection backends (DRP-AI hardware C++ binary).
    - hardware:   Hardware abstractions (Arduino, camera, LiDAR).
    - gui:        GTK3 interface (main window, settings, map/camera panels).
    - state:      Thread-safe shared state (sensor, settings, nav stores).
    - ipc:        Lock-free struct SHM readers/writers (SPSC channels).
    - utils:      Utilities (distance estimation, logging).

Example:
    from target_nav.state import state
    from target_nav.detectors import DrpBinaryDetector, Detection
    from target_nav.utils import DistanceEstimator
"""

__version__ = '1.1.0'
