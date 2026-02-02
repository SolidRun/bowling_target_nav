#!/bin/bash
# ============================================================================
# V2N Robot Startup Script
# ============================================================================
#
# This script starts the robot drivers and keeps it ready for remote control.
# Run this on V2N to enable PC controller connection.
#
# Usage:
#   ./start_robot.sh           # Start robot drivers (bringup only)
#   ./start_robot.sh --full    # Start with SLAM mapping
#   ./start_robot.sh --stop    # Stop all robot processes
#
# ============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  V2N Robot Startup${NC}"
echo -e "${CYAN}============================================${NC}"

# ============================================================================
# Handle --stop flag
# ============================================================================
if [ "$1" == "--stop" ]; then
    echo "Stopping robot processes..."
    pkill -f "ros2 launch bowling" 2>/dev/null || true
    pkill -f "arduino_driver" 2>/dev/null || true
    pkill -f "rplidar" 2>/dev/null || true
    pkill -f "cartographer" 2>/dev/null || true
    pkill -f "slam_camera" 2>/dev/null || true
    sleep 1
    echo -e "${GREEN}[OK]${NC} Robot stopped"
    exit 0
fi

# ============================================================================
# Step 1: Source ROS2
# ============================================================================
echo ""
echo "Setting up ROS2 environment..."

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}[OK]${NC} ROS2 Humble sourced"
else
    echo -e "${RED}[ERROR]${NC} ROS2 Humble not found at /opt/ros/humble"
    exit 1
fi

# Source workspace
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo -e "${GREEN}[OK]${NC} Workspace sourced"
else
    echo -e "${YELLOW}[WARN]${NC} Workspace not found, trying to build..."
    cd ~/ros2_ws
    colcon build --packages-select bowling_target_nav --symlink-install
    source install/setup.bash
fi

# ============================================================================
# Step 2: Set ROS2 environment for WiFi
# ============================================================================
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
echo -e "${GREEN}[OK]${NC} ROS_DOMAIN_ID=0"

# ============================================================================
# Step 3: Kill existing processes
# ============================================================================
echo ""
echo "Cleaning up existing processes..."
pkill -f "ros2 launch bowling" 2>/dev/null || true
pkill -f "arduino_driver" 2>/dev/null || true
pkill -f "rplidar" 2>/dev/null || true
sleep 1

# Release devices
fuser -k /dev/ttyACM0 2>/dev/null || true
fuser -k /dev/ttyUSB0 2>/dev/null || true
sleep 1
echo -e "${GREEN}[OK]${NC} Cleanup complete"

# ============================================================================
# Step 4: Check hardware
# ============================================================================
echo ""
echo "Checking hardware..."

# Check Arduino
if [ -e /dev/ttyACM0 ]; then
    echo -e "${GREEN}[OK]${NC} Arduino found at /dev/ttyACM0"
else
    echo -e "${YELLOW}[WARN]${NC} Arduino not found at /dev/ttyACM0"
fi

# Check LiDAR
if [ -e /dev/ttyUSB0 ]; then
    echo -e "${GREEN}[OK]${NC} LiDAR found at /dev/ttyUSB0"
else
    echo -e "${YELLOW}[WARN]${NC} LiDAR not found at /dev/ttyUSB0"
fi

# ============================================================================
# Step 5: Start robot
# ============================================================================
echo ""
echo -e "${CYAN}============================================${NC}"

if [ "$1" == "--full" ]; then
    echo -e "${CYAN}  Starting FULL system (Bringup + SLAM)${NC}"
    echo -e "${CYAN}============================================${NC}"
    echo ""
    echo "Starting bringup + mapping..."
    ros2 launch bowling_target_nav bringup.launch.py &
    sleep 5
    ros2 launch bowling_target_nav mapping.launch.py &
else
    echo -e "${CYAN}  Starting Bringup (Robot Ready for Control)${NC}"
    echo -e "${CYAN}============================================${NC}"
    echo ""
    echo "Starting robot drivers..."
    echo "  - Arduino driver (motor control)"
    echo "  - LiDAR driver"
    echo "  - Odometry"
    echo ""
    ros2 launch bowling_target_nav bringup.launch.py
fi
