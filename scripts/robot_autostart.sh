#!/bin/bash
# ============================================================================
# V2N Robot Auto-Start Script (FAST VERSION)
# ============================================================================
# Starts all robot services on boot - optimized for fast startup
# ============================================================================

LOG_FILE="/var/log/robot_autostart.log"
exec > >(tee -a "$LOG_FILE") 2>&1

echo ""
echo "============================================"
echo "  V2N Robot Auto-Start (Fast)"
echo "  $(date)"
echo "============================================"

# Quick initial wait
sleep 2

# Wait for hardware (max 10 seconds)
echo "Checking hardware..."
for i in {1..10}; do
    if [ -e /dev/ttyACM0 ] || [ -e /dev/ttyUSB0 ]; then
        break
    fi
    sleep 1
done

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || { echo "ROS2 not found"; exit 1; }
source ~/ros2_ws/install/setup.bash 2>/dev/null || { echo "Workspace not found"; exit 1; }

# ROS2 networking
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export XDG_RUNTIME_DIR=/run/user/996
export WAYLAND_DISPLAY=wayland-1

# Quick cleanup
pkill -f "ros2 launch bowling" 2>/dev/null
pkill -f "arduino_driver\|rplidar\|cartographer\|main_gui\|app_yolo_cam" 2>/dev/null
fuser -k /dev/ttyACM0 /dev/ttyUSB0 2>/dev/null
rm -f /dev/shm/v2n_camera
sleep 1

# Hardware status
[ -e /dev/ttyACM0 ] && echo "[OK] Arduino" || echo "[WARN] No Arduino"
[ -e /dev/ttyUSB0 ] && echo "[OK] LiDAR" || echo "[WARN] No LiDAR"
[ -e /dev/video0 ] && echo "[OK] Camera" || echo "[WARN] No Camera"

# Wait for Weston (max 10 seconds)
echo "Waiting for display..."
for i in {1..10}; do
    [ -e /run/user/996/wayland-1 ] || [ -e /run/user/0/wayland-0 ] && break
    sleep 1
done

# Cleanup on exit
cleanup() {
    pkill -f "ros2 launch bowling" 2>/dev/null
    pkill -f "arduino_driver\|rplidar\|cartographer" 2>/dev/null
}
trap cleanup EXIT

# Start services (GUI is started separately via bowling-launcher.service)
echo ""
echo "Starting robot services..."

ros2 launch bowling_target_nav bringup.launch.py &
sleep 3

ros2 launch bowling_target_nav mapping.launch.py &
sleep 2

echo ""
echo "============================================"
echo "  Robot Ready! (GUI via launcher button)"
echo "============================================"
echo "Topics: /cmd_vel /arduino/cmd /arduino/status /scan /odom /map"
echo ""

# Keep running
wait
