#!/bin/bash
# Run bowling target vision node with video display

# Setup display for V2N Wayland
export XDG_RUNTIME_DIR=/run/user/996
export WAYLAND_DISPLAY=wayland-1

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Run vision node
ros2 launch bowling_target_nav bowling_target_nav.launch.py enable_follower:=false
