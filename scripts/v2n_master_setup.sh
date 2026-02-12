#!/bin/bash
# =============================================================================
# V2N Master Setup Script
# =============================================================================
#
# ONE SCRIPT TO DO EVERYTHING on RZ/V2N robot:
#   1. Build the ROS2 package
#   2. Install systemd autostart service
#   3. Start the robot GUI with SLAM + Camera + Navigation
#
# Usage:
#   ./v2n_master_setup.sh              # Full setup + start GUI
#   ./v2n_master_setup.sh --build      # Only build package
#   ./v2n_master_setup.sh --service    # Only install service
#   ./v2n_master_setup.sh --start      # Only start GUI
#   ./v2n_master_setup.sh --stop       # Stop everything
#   ./v2n_master_setup.sh --status     # Check status
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
PKG_NAME="bowling_target_nav"
WS_DIR="$HOME/ros2_ws"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# =============================================================================
# Helper Functions
# =============================================================================

print_header() {
    echo ""
    echo -e "${CYAN}============================================${NC}"
    echo -e "${CYAN}  $1${NC}"
    echo -e "${CYAN}============================================${NC}"
    echo ""
}

print_ok() {
    echo -e "${GREEN}[OK]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_ros2() {
    if [ ! -d /opt/ros/humble ]; then
        print_error "ROS2 Humble not found at /opt/ros/humble"
        exit 1
    fi
    source /opt/ros/humble/setup.bash
    print_ok "ROS2 Humble found"
}

check_hardware() {
    echo "Checking hardware..."

    if [ -e /dev/ttyACM0 ]; then
        print_ok "Arduino found on /dev/ttyACM0"
    else
        print_warn "Arduino not found on /dev/ttyACM0"
    fi

    if [ -e /dev/ttyUSB0 ]; then
        print_ok "LiDAR found on /dev/ttyUSB0"
    else
        print_warn "LiDAR not found on /dev/ttyUSB0"
    fi

    if [ -e /dev/video0 ]; then
        print_ok "Camera found on /dev/video0"
    else
        print_warn "Camera not found on /dev/video0"
    fi
}

# =============================================================================
# Build Package
# =============================================================================

do_build() {
    print_header "Building Package"

    check_ros2

    # Create workspace if needed
    if [ ! -d "$WS_DIR/src" ]; then
        echo "Creating workspace at $WS_DIR..."
        mkdir -p "$WS_DIR/src"
    fi

    # Make sure package is in workspace
    if [ ! -d "$WS_DIR/src/$PKG_NAME" ]; then
        echo "Copying package to workspace..."
        cp -r "$PKG_DIR" "$WS_DIR/src/"
    fi

    # Make scripts executable
    echo "Making scripts executable..."
    chmod +x "$WS_DIR/src/$PKG_NAME/scripts/"*.sh 2>/dev/null || true
    chmod +x "$WS_DIR/src/$PKG_NAME/scripts/"*.py 2>/dev/null || true
    chmod +x "$WS_DIR/src/$PKG_NAME/tools/"*.sh 2>/dev/null || true
    chmod +x "$WS_DIR/src/$PKG_NAME/tools/"*.py 2>/dev/null || true

    # Build
    echo "Building package (this may take a while)..."
    cd "$WS_DIR"

    # Clean old build if exists
    rm -rf "build/$PKG_NAME" "install/$PKG_NAME" 2>/dev/null || true

    colcon build --packages-select "$PKG_NAME" --symlink-install

    print_ok "Package built successfully"

    # Verify
    source "$WS_DIR/install/setup.bash"
    if ros2 pkg list 2>/dev/null | grep -q "^$PKG_NAME$"; then
        print_ok "Package registered with ROS2"
    else
        print_error "Package not registered with ROS2"
        exit 1
    fi
}

# =============================================================================
# Install Service
# =============================================================================

do_service() {
    print_header "Installing Autostart Service"

    SERVICE_FILE="/etc/systemd/system/robot.service"

    echo "Creating systemd service..."

    cat > /tmp/robot.service << 'SERVICEEOF'
[Unit]
Description=V2N Robot Control (SLAM + Camera + Navigation)
After=network.target
Wants=network.target

[Service]
Type=simple
User=root
Environment="HOME=/root"
Environment="DISPLAY=:0"
Environment="WAYLAND_DISPLAY=wayland-0"
Environment="XDG_RUNTIME_DIR=/run"
ExecStartPre=/bin/sleep 5
ExecStart=/root/ros2_ws/src/bowling_target_nav/scripts/robot_start.sh
ExecStop=/root/ros2_ws/src/bowling_target_nav/scripts/robot_stop.sh
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
SERVICEEOF

    cp /tmp/robot.service "$SERVICE_FILE"
    chmod 644 "$SERVICE_FILE"

    # Create start script
    cat > "$WS_DIR/src/$PKG_NAME/scripts/robot_start.sh" << 'STARTEOF'
#!/bin/bash
# Robot startup script for systemd

export HOME=/root
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/run

# Source ROS2
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# ROS2 DDS config
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

# Kill any existing processes
pkill -f "arduino_driver" 2>/dev/null || true
pkill -f "rplidar" 2>/dev/null || true
pkill -f "main_gui" 2>/dev/null || true
pkill -f "cartographer" 2>/dev/null || true
sleep 2

# Release serial ports
fuser -k /dev/ttyACM0 2>/dev/null || true
fuser -k /dev/ttyUSB0 2>/dev/null || true
sleep 1

echo "[Robot] Starting robot nodes..."

# Start bringup (LiDAR, Arduino, Odometry)
ros2 launch bowling_target_nav bringup.launch.py &
sleep 5

# Start SLAM mapping
ros2 launch bowling_target_nav mapping.launch.py &
sleep 3

# Start main GUI with navigation
ros2 run bowling_target_nav main_gui &

echo "[Robot] All nodes started"
wait
STARTEOF

    chmod +x "$WS_DIR/src/$PKG_NAME/scripts/robot_start.sh"

    # Create stop script
    cat > "$WS_DIR/src/$PKG_NAME/scripts/robot_stop.sh" << 'STOPEOF'
#!/bin/bash
# Robot stop script for systemd

echo "[Robot] Stopping all nodes..."

pkill -f "main_gui" 2>/dev/null || true
pkill -f "cartographer" 2>/dev/null || true
pkill -f "rplidar" 2>/dev/null || true
pkill -f "arduino_driver" 2>/dev/null || true
pkill -f "odometry_node" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true

sleep 2

# Release serial ports
fuser -k /dev/ttyACM0 2>/dev/null || true
fuser -k /dev/ttyUSB0 2>/dev/null || true

echo "[Robot] All nodes stopped"
STOPEOF

    chmod +x "$WS_DIR/src/$PKG_NAME/scripts/robot_stop.sh"

    # Reload systemd
    systemctl daemon-reload

    # Enable service
    systemctl enable robot.service

    print_ok "Service installed and enabled"
    echo ""
    echo "Service commands:"
    echo "  systemctl start robot     # Start robot"
    echo "  systemctl stop robot      # Stop robot"
    echo "  systemctl restart robot   # Restart robot"
    echo "  systemctl status robot    # Check status"
    echo "  journalctl -u robot -f    # View logs"
}

# =============================================================================
# Start GUI
# =============================================================================

do_start() {
    print_header "Starting Robot"

    check_ros2
    check_hardware

    # Source workspace
    if [ -f "$WS_DIR/install/setup.bash" ]; then
        source "$WS_DIR/install/setup.bash"
    else
        print_error "Workspace not built. Run: $0 --build"
        exit 1
    fi

    # Setup display for Wayland
    export DISPLAY=:0
    export WAYLAND_DISPLAY=wayland-0
    export XDG_RUNTIME_DIR=/run

    # ROS2 DDS config
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DOMAIN_ID=0

    # Stop existing processes
    echo "Stopping existing processes..."
    pkill -f "main_gui" 2>/dev/null || true
    pkill -f "arduino_driver" 2>/dev/null || true
    pkill -f "rplidar" 2>/dev/null || true
    pkill -f "cartographer" 2>/dev/null || true
    sleep 2

    # Release serial ports
    fuser -k /dev/ttyACM0 2>/dev/null || true
    fuser -k /dev/ttyUSB0 2>/dev/null || true
    sleep 1

    echo ""
    print_ok "Starting robot nodes..."
    echo ""

    # Start bringup (LiDAR, Arduino, Odometry)
    echo "[1/3] Starting bringup (LiDAR, Arduino, Odometry)..."
    ros2 launch bowling_target_nav bringup.launch.py &
    BRINGUP_PID=$!
    sleep 5

    # Start SLAM mapping
    echo "[2/3] Starting SLAM mapping..."
    ros2 launch bowling_target_nav mapping.launch.py &
    MAPPING_PID=$!
    sleep 3

    # Start main GUI
    echo "[3/3] Starting main GUI..."
    ros2 run bowling_target_nav main_gui

    # Cleanup when GUI exits
    echo "GUI closed, cleaning up..."
    kill $MAPPING_PID 2>/dev/null || true
    kill $BRINGUP_PID 2>/dev/null || true
}

# =============================================================================
# Stop Everything
# =============================================================================

do_stop() {
    print_header "Stopping Robot"

    # Stop service if running
    systemctl stop robot 2>/dev/null || true

    # Kill all robot processes
    pkill -f "main_gui" 2>/dev/null || true
    pkill -f "slam_camera_gui" 2>/dev/null || true
    pkill -f "cartographer" 2>/dev/null || true
    pkill -f "rplidar" 2>/dev/null || true
    pkill -f "arduino_driver" 2>/dev/null || true
    pkill -f "odometry_node" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "nav2" 2>/dev/null || true

    sleep 2

    # Release serial ports
    fuser -k /dev/ttyACM0 2>/dev/null || true
    fuser -k /dev/ttyUSB0 2>/dev/null || true

    print_ok "All robot processes stopped"
}

# =============================================================================
# Check Status
# =============================================================================

do_status() {
    print_header "Robot Status"

    echo "Service status:"
    systemctl status robot --no-pager 2>/dev/null || echo "  Service not installed"
    echo ""

    echo "Running processes:"
    pgrep -a -f "main_gui\|arduino_driver\|rplidar\|cartographer" 2>/dev/null || echo "  No robot processes running"
    echo ""

    echo "Hardware:"
    check_hardware
    echo ""

    echo "ROS2 topics:"
    if command -v ros2 &> /dev/null; then
        source /opt/ros/humble/setup.bash 2>/dev/null
        source "$WS_DIR/install/setup.bash" 2>/dev/null
        ros2 topic list 2>/dev/null | head -20 || echo "  Cannot list topics"
    fi
}

# =============================================================================
# Full Setup (Default)
# =============================================================================

do_full_setup() {
    print_header "V2N Full Setup"

    echo "This will:"
    echo "  1. Build the ROS2 package"
    echo "  2. Install autostart service"
    echo "  3. Start the robot GUI"
    echo ""

    do_build
    do_service

    print_header "Setup Complete!"

    echo "The robot is now configured to start automatically on boot."
    echo ""
    echo "Starting GUI now..."
    echo ""

    do_start
}

# =============================================================================
# Main
# =============================================================================

case "${1:-}" in
    --build)
        do_build
        ;;
    --service)
        do_service
        ;;
    --start)
        do_start
        ;;
    --stop)
        do_stop
        ;;
    --status)
        do_status
        ;;
    --help|-h)
        echo "V2N Master Setup Script"
        echo ""
        echo "Usage: $0 [OPTION]"
        echo ""
        echo "Options:"
        echo "  (none)      Full setup: build + service + start"
        echo "  --build     Only build the ROS2 package"
        echo "  --service   Only install systemd service"
        echo "  --start     Only start the robot GUI"
        echo "  --stop      Stop all robot processes"
        echo "  --status    Check robot status"
        echo "  --help      Show this help"
        ;;
    *)
        do_full_setup
        ;;
esac
