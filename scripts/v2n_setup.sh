#!/bin/bash
# ============================================================================
# V2N Robot Complete Setup Script
# ============================================================================
#
# This script sets up everything needed on a new V2N device:
# 1. Verifies ROS2 environment
# 2. Copies package to workspace (if needed)
# 3. Makes all scripts executable
# 4. Builds the package
# 5. Checks hardware connections
# 6. Optionally installs auto-start service
#
# Usage (from your PC after copying this package to V2N):
#   scp -r bowling_target_nav root@192.168.50.1:~/ros2_ws/src/
#   ssh root@192.168.50.1 "cd ~/ros2_ws/src/bowling_target_nav/scripts && chmod +x *.sh && ./v2n_setup.sh"
#
# Or with autostart enabled:
#   ssh root@192.168.50.1 "cd ~/ros2_ws/src/bowling_target_nav/scripts && chmod +x *.sh && ./v2n_setup.sh --autostart"
#
# ============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# Parse arguments
INSTALL_AUTOSTART=false
if [ "$1" == "--autostart" ] || [ "$1" == "-a" ]; then
    INSTALL_AUTOSTART=true
fi

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  V2N Robot Complete Setup${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""

# ============================================================================
# Step 1: Check environment
# ============================================================================
echo "Step 1: Checking environment..."

# Check if running on V2N
if [ ! -d /opt/ros/humble ]; then
    echo -e "${RED}[ERROR]${NC} ROS2 Humble not found. Is this a V2N device?"
    exit 1
fi
echo -e "${GREEN}[OK]${NC} ROS2 Humble found"

# Check workspace
if [ ! -d ~/ros2_ws ]; then
    echo "Creating workspace..."
    mkdir -p ~/ros2_ws/src
fi
echo -e "${GREEN}[OK]${NC} Workspace exists at ~/ros2_ws"

# ============================================================================
# Step 2: Source ROS2
# ============================================================================
echo ""
echo "Step 2: Sourcing ROS2..."
source /opt/ros/humble/setup.bash
echo -e "${GREEN}[OK]${NC} ROS2 sourced"

# ============================================================================
# Step 3: Find and verify package location
# ============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

echo ""
echo "Step 3: Package location: $PKG_DIR"

# Check if package is in the right place
if [ ! -f "$PKG_DIR/package.xml" ]; then
    echo -e "${RED}[ERROR]${NC} package.xml not found. Is this script in the right location?"
    exit 1
fi

# Move to workspace if not there
if [[ "$PKG_DIR" != *"ros2_ws/src"* ]]; then
    echo "Moving package to workspace..."
    cp -r "$PKG_DIR" ~/ros2_ws/src/
    PKG_DIR=~/ros2_ws/src/bowling_target_nav
    SCRIPT_DIR="$PKG_DIR/scripts"
    echo -e "${GREEN}[OK]${NC} Package copied to workspace"
else
    echo -e "${GREEN}[OK]${NC} Package already in workspace"
fi

# ============================================================================
# Step 4: Make scripts executable
# ============================================================================
echo ""
echo "Step 4: Making scripts executable..."
chmod +x "$PKG_DIR/scripts/"*.sh 2>/dev/null || true
chmod +x "$PKG_DIR/scripts/"*.py 2>/dev/null || true
chmod +x "$PKG_DIR/tools/"*.sh 2>/dev/null || true
chmod +x "$PKG_DIR/tools/"*.py 2>/dev/null || true
echo -e "${GREEN}[OK]${NC} Scripts are executable"

# ============================================================================
# Step 5: Build package
# ============================================================================
echo ""
echo "Step 5: Building package..."
cd ~/ros2_ws

# Clean old build if exists
rm -rf build/bowling_target_nav install/bowling_target_nav 2>/dev/null || true

# Build
if colcon build --packages-select bowling_target_nav --symlink-install; then
    echo -e "${GREEN}[OK]${NC} Package built successfully"
else
    echo -e "${RED}[ERROR]${NC} Build failed"
    exit 1
fi

# Source workspace
source install/setup.bash

# ============================================================================
# Step 6: Check hardware
# ============================================================================
echo ""
echo "Step 6: Checking hardware..."

HARDWARE_OK=true

# Check Arduino
if [ -e /dev/ttyACM0 ]; then
    echo -e "${GREEN}[OK]${NC} Arduino found at /dev/ttyACM0"
else
    echo -e "${YELLOW}[WARN]${NC} Arduino not found at /dev/ttyACM0"
    HARDWARE_OK=false
fi

# Check LiDAR
if [ -e /dev/ttyUSB0 ]; then
    echo -e "${GREEN}[OK]${NC} LiDAR found at /dev/ttyUSB0"
else
    echo -e "${YELLOW}[WARN]${NC} LiDAR not found at /dev/ttyUSB0"
fi

# Check camera
if [ -e /dev/video0 ]; then
    echo -e "${GREEN}[OK]${NC} Camera found at /dev/video0"
else
    echo -e "${YELLOW}[WARN]${NC} Camera not found at /dev/video0"
fi

# ============================================================================
# Step 7: Verify ROS2 package registration
# ============================================================================
echo ""
echo "Step 7: Verifying ROS2 registration..."

if ros2 pkg list | grep -q bowling_target_nav; then
    echo -e "${GREEN}[OK]${NC} Package registered with ROS2"
else
    echo -e "${RED}[ERROR]${NC} Package not found in ROS2"
    exit 1
fi

# ============================================================================
# Step 8: Install auto-start (if requested)
# ============================================================================
echo ""
echo "Step 8: Auto-start service..."

if [ "$INSTALL_AUTOSTART" = true ]; then
    echo "Installing auto-start service..."

    # Copy service file
    cp "$SCRIPT_DIR/robot.service" /etc/systemd/system/robot.service

    # Reload systemd
    systemctl daemon-reload

    # Enable and start service
    systemctl enable robot
    systemctl start robot

    echo -e "${GREEN}[OK]${NC} Auto-start service installed and started"
else
    echo -e "${YELLOW}[SKIP]${NC} Auto-start not requested (use --autostart flag)"
fi

# ============================================================================
# Setup Complete
# ============================================================================
echo ""
echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  Setup Complete!${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""

if [ "$HARDWARE_OK" = false ]; then
    echo -e "${YELLOW}NOTE: Some hardware was not detected. Check connections.${NC}"
    echo ""
fi

echo "Commands:"
echo ""
echo "  Start robot (manual):"
echo "    ~/ros2_ws/src/bowling_target_nav/scripts/start_robot.sh"
echo ""
echo "  Start with SLAM mapping:"
echo "    ~/ros2_ws/src/bowling_target_nav/scripts/start_robot.sh --full"
echo ""
echo "  Start SLAM + Camera GUI:"
echo "    ~/ros2_ws/src/bowling_target_nav/scripts/run_slam_camera.sh"
echo ""

if [ "$INSTALL_AUTOSTART" = true ]; then
    echo "  Auto-start service:"
    echo "    systemctl status robot    # Check status"
    echo "    systemctl restart robot   # Restart"
    echo "    journalctl -u robot -f    # View logs"
    echo ""
else
    echo "  Enable auto-start on boot:"
    echo "    cd ~/ros2_ws/src/bowling_target_nav/scripts"
    echo "    ./install_autostart.sh"
    echo ""
fi

echo "From PC (connect to V2N WiFi first):"
echo "  cd tools/ && ./run_controller.sh"
echo ""
