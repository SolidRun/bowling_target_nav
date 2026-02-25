"""
ROS2 Nodes for Bowling Target Navigation
=========================================

Each node is a standalone entry point with its own dependencies.
Use ``ros2 run bowling_target_nav <node_name>`` or import directly.

Active nodes:
    main_gui             Unified GUI with SLAM map, camera, navigation
    arduino_driver_node  Motor control via serial Arduino bridge
    odometry_node        Wheel encoder odometry + TF broadcast
    vision_node          Camera capture + YOLO detection publisher
    target_follower_node Visual-servoing target follower
    autonomous_explorer  Frontier-based autonomous exploration
    target_gui_node      Standalone detection GUI
    map_viewer_node      SLAM map viewer
    slam_camera_gui      Combined SLAM + camera display
"""
