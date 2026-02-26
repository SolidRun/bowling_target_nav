#!/bin/bash
# ============================================================================
# Bowling Target Nav GUI Launcher
# ============================================================================
# Kills all related processes before starting a clean GUI instance.
# Called by bowling_launcher.py's Start button.
# ============================================================================

# Source ROS2
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Kill ALL related processes
pkill -f "main_gui" 2>/dev/null
pkill -f "app_yolo_cam" 2>/dev/null
sleep 0.5

# Clean shared memory and free camera
rm -f /dev/shm/v2n_camera
fuser -k /dev/video0 2>/dev/null
sleep 0.3

# Launch GUI
export HOME=/root
exec ros2 run bowling_target_nav main_gui
