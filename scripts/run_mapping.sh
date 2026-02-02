#!/bin/bash
# Quick start script for SLAM mapping on V2N
# Starts: RPLidar A1 + Arduino + Odometry + Cartographer
# Usage: ~/run_mapping.sh

export XDG_RUNTIME_DIR=/run/user/996
export WAYLAND_DISPLAY=wayland-1

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=========================================="
echo "  SLAM Mapping - Bowling Target Nav"
echo "=========================================="
echo "  Using RPLidar A1 (115200 baud)"
echo "=========================================="
echo ""

# Cleanup old processes
echo "Cleaning up old processes..."
pkill -9 -f rplidar 2>/dev/null
pkill -9 -f cartographer 2>/dev/null
pkill -9 -f arduino_driver 2>/dev/null
pkill -9 -f odometry_node 2>/dev/null
pkill -9 -f map_viewer 2>/dev/null
pkill -9 -f ros2 2>/dev/null
sleep 1
fuser -k /dev/ttyUSB0 2>/dev/null
fuser -k /dev/ttyACM0 2>/dev/null
fuser -k /dev/ttyACM1 2>/dev/null
sleep 2
echo "Cleanup done."
echo ""

echo "Starting: RPLidar + Arduino + Cartographer"
echo "Press Ctrl+C to stop"
echo ""

ros2 launch bowling_target_nav slam.launch.py show_map:=true
