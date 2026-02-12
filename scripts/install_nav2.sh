#!/bin/bash
# =============================================================================
# Nav2 Installation Guide for V2N (Yocto-based)
# =============================================================================
#
# The V2N uses Yocto, so Nav2 packages must be added to the Yocto image build.
# This script checks if Nav2 is installed and provides instructions if not.
#
# =============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  Nav2 Navigation - Installation Check${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""

# Check if ROS2 is installed
if [ ! -d /opt/ros/humble ]; then
    echo -e "${RED}[ERROR]${NC} ROS2 Humble not found!"
    exit 1
fi

source /opt/ros/humble/setup.bash

# Check for Nav2 packages
echo "Checking for Nav2 packages..."
echo ""

NAV2_MISSING=false

check_package() {
    if ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        echo -e "${GREEN}[OK]${NC} $1"
    else
        echo -e "${RED}[MISSING]${NC} $1"
        NAV2_MISSING=true
    fi
}

check_package "nav2_bringup"
check_package "nav2_bt_navigator"
check_package "nav2_controller"
check_package "nav2_planner"
check_package "nav2_behaviors"
check_package "nav2_costmap_2d"
check_package "nav2_lifecycle_manager"
check_package "nav2_map_server"
check_package "nav2_amcl"
check_package "nav2_velocity_smoother"
check_package "nav2_collision_monitor"
check_package "robot_state_publisher"

echo ""

if [ "$NAV2_MISSING" = true ]; then
    echo -e "${YELLOW}============================================${NC}"
    echo -e "${YELLOW}  Nav2 packages are MISSING${NC}"
    echo -e "${YELLOW}============================================${NC}"
    echo ""
    echo "Since V2N uses Yocto, you need to add Nav2 to your Yocto build."
    echo ""
    echo "Add these packages to your Yocto image recipe (local.conf or image recipe):"
    echo ""
    echo -e "${CYAN}IMAGE_INSTALL:append = \" \\"
    echo "    ros-humble-navigation2 \\"
    echo "    ros-humble-nav2-bringup \\"
    echo "    ros-humble-nav2-bt-navigator \\"
    echo "    ros-humble-nav2-controller \\"
    echo "    ros-humble-nav2-planner \\"
    echo "    ros-humble-nav2-behaviors \\"
    echo "    ros-humble-nav2-costmap-2d \\"
    echo "    ros-humble-nav2-lifecycle-manager \\"
    echo "    ros-humble-nav2-map-server \\"
    echo "    ros-humble-nav2-amcl \\"
    echo "    ros-humble-nav2-velocity-smoother \\"
    echo "    ros-humble-nav2-collision-monitor \\"
    echo "    ros-humble-nav2-smoother \\"
    echo "    ros-humble-nav2-waypoint-follower \\"
    echo "    ros-humble-robot-state-publisher \\"
    echo "    ros-humble-dwb-core \\"
    echo "    ros-humble-dwb-plugins \\"
    echo "    ros-humble-nav2-navfn-planner \\"
    echo "\"${NC}"
    echo ""
    echo "Then rebuild and reflash your Yocto image."
    echo ""
    exit 1
else
    echo -e "${GREEN}============================================${NC}"
    echo -e "${GREEN}  All Nav2 packages are installed!${NC}"
    echo -e "${GREEN}============================================${NC}"
    echo ""
    echo "You can now use Nav2 navigation:"
    echo ""
    echo "  # Start robot drivers"
    echo "  ros2 launch bowling_target_nav bringup.launch.py"
    echo ""
    echo "  # Start navigation"
    echo "  ros2 launch bowling_target_nav navigation.launch.py"
    echo ""
    echo "  # Send a navigation goal"
    echo "  ./nav_to_pose.sh 1.0 0.5"
    echo ""
fi
