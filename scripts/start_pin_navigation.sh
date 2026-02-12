#!/bin/bash
# =============================================================================
# Start Automatic Bowling Pin Navigation
# =============================================================================
#
# This script starts all nodes needed for automatic navigation to detected
# bowling pins using YOLO + Nav2.
#
# Usage:
#   ./start_pin_navigation.sh           # Start all nodes
#   ./start_pin_navigation.sh --stop    # Stop all nodes
#   ./start_pin_navigation.sh --direct  # Use direct control instead of Nav2
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# Default mode
FOLLOWER_MODE="nav2"

# Parse arguments
if [ "$1" == "--stop" ]; then
    echo -e "${YELLOW}Stopping all pin navigation nodes...${NC}"
    pkill -f "vision_node" 2>/dev/null || true
    pkill -f "target_follower_node" 2>/dev/null || true
    pkill -f "nav2" 2>/dev/null || true
    pkill -f "bringup.launch" 2>/dev/null || true
    pkill -f "navigation.launch" 2>/dev/null || true
    echo -e "${GREEN}All nodes stopped.${NC}"
    exit 0
fi

if [ "$1" == "--direct" ]; then
    FOLLOWER_MODE="direct"
    echo -e "${YELLOW}Using DIRECT control mode (no obstacle avoidance)${NC}"
fi

# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  Automatic Bowling Pin Navigation${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""
echo -e "Mode: ${GREEN}${FOLLOWER_MODE}${NC}"
echo ""

# Check hardware
echo "Checking hardware..."
if [ ! -e /dev/ttyACM0 ]; then
    echo -e "${RED}[WARNING]${NC} Arduino not found on /dev/ttyACM0"
fi
if [ ! -e /dev/ttyUSB0 ]; then
    echo -e "${RED}[WARNING]${NC} LiDAR not found on /dev/ttyUSB0"
fi
if [ ! -e /dev/video0 ]; then
    echo -e "${RED}[WARNING]${NC} Camera not found on /dev/video0"
fi

# Kill any existing processes
echo "Cleaning up existing processes..."
pkill -f "vision_node" 2>/dev/null || true
pkill -f "target_follower_node" 2>/dev/null || true
sleep 1

echo ""
echo -e "${GREEN}Starting nodes...${NC}"
echo ""

# Start bringup (LiDAR, Arduino, Odometry)
echo "[1/4] Starting robot bringup..."
ros2 launch bowling_target_nav bringup.launch.py &
BRINGUP_PID=$!
sleep 5

# Start Nav2 navigation (only if using nav2 mode)
if [ "$FOLLOWER_MODE" == "nav2" ]; then
    echo "[2/4] Starting Nav2 navigation..."
    ros2 launch bowling_target_nav navigation.launch.py &
    NAV2_PID=$!
    sleep 8
else
    echo "[2/4] Skipping Nav2 (direct mode)"
fi

# Start vision node (YOLO detection)
echo "[3/4] Starting vision node (YOLO detection)..."
ros2 run bowling_target_nav vision_node &
VISION_PID=$!
sleep 2

# Start target follower
echo "[4/4] Starting target follower (mode: ${FOLLOWER_MODE})..."
ros2 run bowling_target_nav target_follower_node --ros-args -p mode:=${FOLLOWER_MODE} &
FOLLOWER_PID=$!
sleep 1

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}  All nodes started!${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "The robot will now:"
echo "  1. Detect bowling pins using YOLO"
echo "  2. Estimate distance to detected pins"
echo "  3. Navigate to the nearest pin using Nav2"
echo ""
echo "Topics to monitor:"
echo "  ros2 topic echo /target_pose           # Detected target"
echo "  ros2 topic echo /target_follower/state # Follower state"
echo ""
echo -e "Press ${YELLOW}Ctrl+C${NC} to stop all nodes"
echo ""

# Wait for Ctrl+C
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down...${NC}"
    kill $VISION_PID 2>/dev/null || true
    kill $FOLLOWER_PID 2>/dev/null || true
    kill $NAV2_PID 2>/dev/null || true
    kill $BRINGUP_PID 2>/dev/null || true
    pkill -f "vision_node" 2>/dev/null || true
    pkill -f "target_follower_node" 2>/dev/null || true
    echo -e "${GREEN}Done.${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Keep running
wait
