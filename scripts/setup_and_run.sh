#!/bin/bash
# ============================================================================
# One-Command Setup and Run for V2N SLAM + Camera GUI
# ============================================================================
# Usage: ./setup_and_run.sh
#
# This script:
# 1. Kills all old processes
# 2. Rebuilds the package if needed
# 3. Starts LiDAR, Arduino, Odometry
# 4. Starts Cartographer SLAM
# 5. Launches fullscreen GUI with camera + YOLO detection
# ============================================================================

set -e

# Display environment
export XDG_RUNTIME_DIR=/run/user/996
export WAYLAND_DISPLAY=wayland-1

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=========================================="
echo "  V2N SLAM + Camera + YOLO Detection"
echo "  One-Command Setup and Run"
echo -e "==========================================${NC}"
echo ""

# Source ROS2
echo -e "${YELLOW}[1/6] Sourcing ROS2...${NC}"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Kill old processes
echo -e "${YELLOW}[2/6] Cleaning up old processes...${NC}"
pkill -9 -f rplidar 2>/dev/null || true
pkill -9 -f cartographer 2>/dev/null || true
pkill -9 -f slam_camera_gui 2>/dev/null || true
pkill -9 -f map_viewer 2>/dev/null || true
pkill -9 -f arduino_driver 2>/dev/null || true
pkill -9 -f odometry_node 2>/dev/null || true
pkill -9 -f gst-launch 2>/dev/null || true
sleep 1

# Release devices
fuser -k /dev/ttyUSB0 2>/dev/null || true
fuser -k /dev/ttyACM0 2>/dev/null || true
fuser -k /dev/ttyACM1 2>/dev/null || true
fuser -k /dev/video0 2>/dev/null || true
sleep 2
echo -e "${GREEN}   Done${NC}"

# Check if rebuild needed
echo -e "${YELLOW}[3/6] Checking package...${NC}"
if [ ! -d ~/ros2_ws/install/bowling_target_nav ]; then
    echo "   Package not installed, building..."
    cd ~/ros2_ws
    colcon build --packages-select bowling_target_nav
    source ~/ros2_ws/install/setup.bash
fi
echo -e "${GREEN}   Package ready${NC}"

# Trap to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down...${NC}"
    pkill -9 -f rplidar 2>/dev/null || true
    pkill -9 -f cartographer 2>/dev/null || true
    pkill -9 -f arduino_driver 2>/dev/null || true
    pkill -9 -f odometry_node 2>/dev/null || true
    pkill -9 -f slam_camera_gui 2>/dev/null || true
    echo -e "${GREEN}Done.${NC}"
}
trap cleanup EXIT

# Start robot drivers
echo -e "${YELLOW}[4/6] Starting robot drivers (LiDAR, Arduino, Odometry)...${NC}"
ros2 launch bowling_target_nav bringup.launch.py &
BRINGUP_PID=$!
sleep 4

# Check if bringup started
if ! kill -0 $BRINGUP_PID 2>/dev/null; then
    echo -e "${RED}   ERROR: Bringup failed to start${NC}"
    exit 1
fi
echo -e "${GREEN}   Drivers running${NC}"

# Start Cartographer SLAM
echo -e "${YELLOW}[5/6] Starting Cartographer SLAM...${NC}"
ros2 launch bowling_target_nav mapping.launch.py &
MAPPING_PID=$!
sleep 3

# Check if mapping started
if ! kill -0 $MAPPING_PID 2>/dev/null; then
    echo -e "${RED}   ERROR: Mapping failed to start${NC}"
    exit 1
fi
echo -e "${GREEN}   SLAM running${NC}"

# Start GUI
echo -e "${YELLOW}[6/6] Starting fullscreen GUI...${NC}"
echo ""
echo -e "${GREEN}=========================================="
echo "  GUI Controls:"
echo "  - Press Q or ESC to quit"
echo "  - Left panel: SLAM map"
echo "  - Right panel: Camera + YOLO detection"
echo -e "==========================================${NC}"
echo ""

# Run the fullscreen GUI (blocks until user quits)
ros2 run bowling_target_nav slam_camera_gui
