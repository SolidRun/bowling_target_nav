#!/bin/bash
# =============================================================================
# Navigate to Pose Script
# =============================================================================
#
# Sends a navigation goal to Nav2.
#
# Usage:
#   ./nav_to_pose.sh X Y           # Navigate to (X, Y) with default orientation
#   ./nav_to_pose.sh X Y YAW       # Navigate to (X, Y) with specified yaw (degrees)
#   ./nav_to_pose.sh 1.0 2.0       # Go to (1.0, 2.0)
#   ./nav_to_pose.sh 1.0 2.0 90    # Go to (1.0, 2.0) facing 90 degrees
#
# Requirements:
#   - Navigation stack running (navigation.launch.py)
#
# =============================================================================

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Check arguments
if [ $# -lt 2 ]; then
    echo "Usage: $0 X Y [YAW_DEGREES]"
    echo ""
    echo "Examples:"
    echo "  $0 1.0 0.0       # Go to (1.0, 0.0)"
    echo "  $0 2.0 1.5 90    # Go to (2.0, 1.5) facing 90 degrees"
    exit 1
fi

X=$1
Y=$2
YAW_DEG=${3:-0}  # Default 0 degrees

# Convert yaw to quaternion (only z and w matter for 2D)
# q.z = sin(yaw/2), q.w = cos(yaw/2)
YAW_RAD=$(echo "scale=6; $YAW_DEG * 3.14159265359 / 180.0" | bc)
QZ=$(echo "scale=6; s($YAW_RAD / 2)" | bc -l)
QW=$(echo "scale=6; c($YAW_RAD / 2)" | bc -l)

echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}  Navigate to Pose${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo -e "Target: X=${X}, Y=${Y}, Yaw=${YAW_DEG}°"
echo ""

# Check if navigation is running
if ! ros2 action list | grep -q "navigate_to_pose"; then
    echo -e "${RED}[ERROR]${NC} Navigation not running!"
    echo "  Start navigation: ros2 launch bowling_target_nav navigation.launch.py"
    exit 1
fi

echo "Sending navigation goal..."
echo ""

# Send goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}}" \
    --feedback

echo ""
echo -e "${GREEN}Navigation complete!${NC}"
