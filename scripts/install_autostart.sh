#!/bin/bash
# ============================================================================
# Install Robot Auto-Start Service
# ============================================================================
#
# This script installs the systemd service that automatically starts
# the robot when V2N boots up.
#
# Usage:
#   ./install_autostart.sh          # Install and enable autostart
#   ./install_autostart.sh --remove # Remove autostart
#   ./install_autostart.sh --status # Check service status
#
# ============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="robot"
SERVICE_FILE="$SCRIPT_DIR/robot.service"
AUTOSTART_SCRIPT="$SCRIPT_DIR/robot_autostart.sh"

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  Robot Auto-Start Installer${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}[ERROR]${NC} This script must be run as root"
    echo "  Usage: sudo ./install_autostart.sh"
    exit 1
fi

# ============================================================================
# Handle --remove flag
# ============================================================================
if [ "$1" == "--remove" ]; then
    echo "Removing robot autostart service..."

    systemctl stop $SERVICE_NAME 2>/dev/null || true
    systemctl disable $SERVICE_NAME 2>/dev/null || true
    rm -f /etc/systemd/system/$SERVICE_NAME.service
    systemctl daemon-reload

    echo -e "${GREEN}[OK]${NC} Autostart service removed"
    echo ""
    echo "The robot will no longer start automatically on boot."
    echo "To start manually, run:"
    echo "  ~/ros2_ws/src/bowling_target_nav/scripts/start_robot.sh"
    exit 0
fi

# ============================================================================
# Handle --status flag
# ============================================================================
if [ "$1" == "--status" ]; then
    echo "Robot service status:"
    echo ""
    systemctl status $SERVICE_NAME --no-pager || echo "Service not installed"
    echo ""
    echo "Recent logs:"
    journalctl -u $SERVICE_NAME -n 20 --no-pager 2>/dev/null || echo "No logs available"
    exit 0
fi

# ============================================================================
# Step 1: Verify files exist
# ============================================================================
echo "Verifying files..."

if [ ! -f "$SERVICE_FILE" ]; then
    echo -e "${RED}[ERROR]${NC} Service file not found: $SERVICE_FILE"
    exit 1
fi

if [ ! -f "$AUTOSTART_SCRIPT" ]; then
    echo -e "${RED}[ERROR]${NC} Autostart script not found: $AUTOSTART_SCRIPT"
    exit 1
fi

echo -e "${GREEN}[OK]${NC} All files present"

# ============================================================================
# Step 2: Make scripts executable
# ============================================================================
echo ""
echo "Making scripts executable..."

chmod +x "$AUTOSTART_SCRIPT"
chmod +x "$SCRIPT_DIR"/*.sh

echo -e "${GREEN}[OK]${NC} Scripts are executable"

# ============================================================================
# Step 3: Install systemd service
# ============================================================================
echo ""
echo "Installing systemd service..."

# Copy service file
cp "$SERVICE_FILE" /etc/systemd/system/$SERVICE_NAME.service

# Reload systemd
systemctl daemon-reload

echo -e "${GREEN}[OK]${NC} Service installed"

# ============================================================================
# Step 4: Enable service
# ============================================================================
echo ""
echo "Enabling service to start on boot..."

systemctl enable $SERVICE_NAME

echo -e "${GREEN}[OK]${NC} Service enabled"

# ============================================================================
# Step 5: Ask about starting now
# ============================================================================
echo ""
echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  Installation Complete!${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""
echo "The robot will now start automatically when V2N boots."
echo ""
echo "Service commands:"
echo "  systemctl start $SERVICE_NAME    # Start service now"
echo "  systemctl stop $SERVICE_NAME     # Stop service"
echo "  systemctl restart $SERVICE_NAME  # Restart service"
echo "  systemctl status $SERVICE_NAME   # Check status"
echo "  journalctl -u $SERVICE_NAME -f   # View logs"
echo ""
echo "To remove autostart:"
echo "  ./install_autostart.sh --remove"
echo ""

read -p "Start the robot service now? [Y/n] " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Nn]$ ]]; then
    echo "Service not started. It will start on next boot."
else
    echo ""
    echo "Starting robot service..."
    systemctl start $SERVICE_NAME
    sleep 2

    if systemctl is-active --quiet $SERVICE_NAME; then
        echo -e "${GREEN}[OK]${NC} Robot service is running!"
        echo ""
        echo "You can now connect from your PC using:"
        echo "  ./run_controller.sh"
    else
        echo -e "${YELLOW}[WARN]${NC} Service may have failed to start. Check status:"
        echo "  systemctl status $SERVICE_NAME"
        echo "  journalctl -u $SERVICE_NAME -n 50"
    fi
fi
