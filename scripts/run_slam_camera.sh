#!/bin/bash
# Run SLAM + Camera + YOLO Detection fullscreen GUI on V2N
# Usage: ./run_slam_camera.sh

export XDG_RUNTIME_DIR=/run/user/996
export WAYLAND_DISPLAY=wayland-1

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=========================================="
echo "  SLAM + Camera + Detection GUI"
echo "  Bowling Target Nav"
echo "=========================================="
echo ""

# Cleanup old processes
echo "Cleaning up old processes..."
pkill -9 -f rplidar 2>/dev/null
pkill -9 -f cartographer 2>/dev/null
pkill -9 -f slam_camera_gui 2>/dev/null
pkill -9 -f map_viewer 2>/dev/null
pkill -9 -f arduino_driver 2>/dev/null
pkill -9 -f odometry_node 2>/dev/null
sleep 1
fuser -k /dev/ttyUSB0 2>/dev/null
fuser -k /dev/ttyACM0 2>/dev/null
fuser -k /dev/ttyACM1 2>/dev/null
# Kill any process using camera
fuser -k /dev/video0 2>/dev/null
pkill -9 -f gst-launch 2>/dev/null
sleep 2
echo "Cleanup done."
echo ""

# Trap to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    pkill -9 -f rplidar 2>/dev/null
    pkill -9 -f cartographer 2>/dev/null
    pkill -9 -f arduino_driver 2>/dev/null
    pkill -9 -f odometry_node 2>/dev/null
    echo "Done."
}
trap cleanup EXIT

echo "Starting robot drivers (LiDAR, Arduino, Odometry)..."
ros2 launch bowling_target_nav bringup.launch.py &
sleep 4

echo "Starting Cartographer SLAM..."
ros2 launch bowling_target_nav mapping.launch.py &
sleep 3

echo ""
echo "Starting fullscreen GUI..."
echo "Press Q or ESC to quit"
echo ""

# Run the fullscreen GUI (this blocks until user quits)
ros2 run bowling_target_nav slam_camera_gui
