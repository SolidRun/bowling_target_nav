"""Application entry points and ROS2 nodes.

All runnable components live here:
    - main.py:              GUI launcher + multiprocess/threading manager.
    - arduino_node.py:      Arduino serial ROS2 node (cmd_vel -> PWM).
    - odometry_node.py:     Encoder odometry ROS2 node (odom + TF).
    - nav_node.py:          Navigation ROS2 node (TF, Navigator, 20 Hz loop).
    - camera_node.py:       Camera + DRP-AI detection ROS2 node.
    - camera_worker.py:     DRP-AI detection pipeline (capture + inference).
    - nav_controller.py:    Navigation state machine (IDLE -> NAVIGATING -> ...).
    - target_tracker.py:    Kalman-filtered target tracking with LiDAR fusion.
    - detection_filters.py: Temporal detection tracking and smoothing.
"""
