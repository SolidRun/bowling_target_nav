#!/bin/bash
# ============================================================================
# V2N Robot Controller - All-in-One Tool
# ============================================================================
#
# Comprehensive tool for:
# - Robot control GUI (ROS2 and standalone)
# - Remote Arduino testing
# - Hardware tests
# - System diagnostics
#
# Usage: ./run_controller.sh [command]
#
# Commands:
#   (none)      - Show interactive menu
#   gui         - Run robot controller GUI
#   arduino     - Remote Arduino control
#   test        - Run hardware tests
#   diag        - System diagnostics
# ============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# Get script directory and package root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

# Default V2N IP
V2N_IP="${V2N_IP:-192.168.50.1}"
V2N_USER="${V2N_USER:-root}"
ARDUINO_PORT="${ARDUINO_PORT:-5555}"

# ============================================================================
# Helper Functions
# ============================================================================

print_header() {
    echo ""
    echo -e "${BOLD}${CYAN}=============================================="
    echo -e "  $1"
    echo -e "==============================================${NC}"
    echo ""
}

print_menu_header() {
    clear
    echo -e "${BOLD}${CYAN}"
    echo "  ╔════════════════════════════════════════════╗"
    echo "  ║     V2N Robot Controller & Test Suite      ║"
    echo "  ╚════════════════════════════════════════════╝"
    echo -e "${NC}"
}

ok() {
    echo -e "${GREEN}[OK]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

err() {
    echo -e "${RED}[ERROR]${NC} $1"
}

info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

check_v2n_connection() {
    echo -n "Checking V2N at $V2N_IP... "
    if ping -c 1 -W 2 "$V2N_IP" &> /dev/null; then
        echo -e "${GREEN}Connected${NC}"
        return 0
    else
        echo -e "${RED}Not reachable${NC}"
        return 1
    fi
}

check_ssh_connection() {
    echo -n "Checking SSH access... "
    if ssh -o ConnectTimeout=3 -o BatchMode=yes "$V2N_USER@$V2N_IP" "echo ok" &>/dev/null; then
        echo -e "${GREEN}OK${NC}"
        return 0
    else
        echo -e "${YELLOW}Needs password or key${NC}"
        return 1
    fi
}

find_ros2() {
    ROS2_PATHS=(
        "/opt/ros/humble/setup.bash"
        "/opt/ros/jazzy/setup.bash"
        "/opt/ros/rolling/setup.bash"
        "/opt/ros/iron/setup.bash"
        "$HOME/ros2_humble/install/setup.bash"
        "$HOME/ros2_ws/install/setup.bash"
    )

    for ros_path in "${ROS2_PATHS[@]}"; do
        if [ -f "$ros_path" ]; then
            echo "$ros_path"
            return 0
        fi
    done
    return 1
}

setup_ros2() {
    ROS2_PATH=$(find_ros2)
    if [ -n "$ROS2_PATH" ]; then
        source "$ROS2_PATH"
        export ROS_DOMAIN_ID=0
        export ROS_LOCALHOST_ONLY=0
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        ok "ROS2 sourced from $ROS2_PATH"
        return 0
    else
        warn "ROS2 not found"
        return 1
    fi
}

# ============================================================================
# Menu Functions
# ============================================================================

show_main_menu() {
    print_menu_header
    echo -e "${BOLD}  Main Menu${NC}"
    echo ""
    echo "  1) Robot Controller GUI (ROS2)"
    echo "  2) Robot Controller (Standalone - no ROS2)"
    echo "  3) Remote Arduino Control"
    echo "  4) Run Hardware Tests"
    echo "  5) System Diagnostics"
    echo "  6) SSH to V2N"
    echo ""
    echo "  c) Configure V2N IP  (current: $V2N_IP)"
    echo "  q) Quit"
    echo ""
    echo -n "  Select option: "
}

show_arduino_menu() {
    print_menu_header
    echo -e "${BOLD}  Arduino Remote Control${NC}"
    echo ""
    echo "  This allows you to control the Arduino motors from your PC."
    echo "  The Arduino is connected to V2N via USB."
    echo ""
    echo "  1) Start Arduino Server on V2N (via SSH)"
    echo "  2) Open Arduino Remote GUI on this PC"
    echo "  3) Quick Motor Test (on V2N via SSH)"
    echo "  4) Open Local GUI (mock mode - no hardware)"
    echo ""
    echo "  b) Back to main menu"
    echo ""
    echo -n "  Select option: "
}

show_test_menu() {
    print_menu_header
    echo -e "${BOLD}  Hardware Tests${NC}"
    echo ""
    echo "  1) Run All Unit Tests (mock)"
    echo "  2) Test Arduino (on V2N via SSH)"
    echo "  3) Test Camera (on V2N via SSH)"
    echo "  4) Test LiDAR (on V2N via SSH)"
    echo "  5) Test Detection System (mock)"
    echo ""
    echo "  b) Back to main menu"
    echo ""
    echo -n "  Select option: "
}

show_diag_menu() {
    print_menu_header
    echo -e "${BOLD}  System Diagnostics${NC}"
    echo ""
    echo "  1) Check V2N Connection"
    echo "  2) Check ROS2 Topics (on V2N)"
    echo "  3) Check Hardware Status (on V2N)"
    echo "  4) View V2N System Info"
    echo "  5) Check Local ROS2 Installation"
    echo ""
    echo "  b) Back to main menu"
    echo ""
    echo -n "  Select option: "
}

# ============================================================================
# Action Functions
# ============================================================================

run_ros2_gui() {
    print_header "Robot Controller GUI (ROS2)"

    if ! setup_ros2; then
        err "ROS2 required. Use standalone mode instead."
        read -p "Press Enter to continue..."
        return
    fi

    # Setup FastDDS for WiFi
    export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_wifi.xml
    cat > /tmp/fastdds_wifi.xml << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <initialPeersList>
                    <locator><udpv4><address>$V2N_IP</address></udpv4></locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOF

    info "Starting ROS2 controller GUI..."
    info "Target: $V2N_IP"
    echo ""

    python3 "$SCRIPT_DIR/pc_robot_controller.py"
}

run_standalone_gui() {
    print_header "Robot Controller (Standalone)"

    STANDALONE="$SCRIPT_DIR/pc_robot_controller_standalone.py"

    if [ ! -f "$STANDALONE" ]; then
        info "Creating standalone controller..."
        create_standalone_controller
    fi

    info "Starting standalone controller..."
    info "Note: Requires rosbridge_server on V2N"
    echo ""

    python3 "$STANDALONE"
}

start_arduino_server() {
    print_header "Starting Arduino Server on V2N"

    if ! check_v2n_connection; then
        err "Cannot reach V2N. Check your WiFi connection."
        read -p "Press Enter to continue..."
        return
    fi

    info "Starting Arduino server on V2N..."
    info "This will run in your terminal. Press Ctrl+C to stop."
    echo ""

    # Copy server script to V2N and run it
    ssh "$V2N_USER@$V2N_IP" "cd ~/ros2_ws/src/bowling_target_nav && python3 test/arduino_remote_server.py --port $ARDUINO_PORT"
}

run_arduino_gui() {
    print_header "Arduino Remote GUI"

    info "Connecting to V2N Arduino server at $V2N_IP:$ARDUINO_PORT"
    info "Make sure arduino_remote_server.py is running on V2N!"
    echo ""

    # Check if GTK is available
    if ! python3 -c "import gi; gi.require_version('Gtk', '3.0')" 2>/dev/null; then
        err "GTK3 not available. Install with: sudo apt install python3-gi"
        read -p "Press Enter to continue..."
        return
    fi

    python3 "$PKG_DIR/test/arduino_remote_gui.py" --host "$V2N_IP" --port "$ARDUINO_PORT"
}

run_arduino_quick_test() {
    print_header "Quick Arduino Motor Test"

    if ! check_v2n_connection; then
        err "Cannot reach V2N."
        read -p "Press Enter to continue..."
        return
    fi

    info "Running motor test on V2N (motors will move briefly!)..."
    echo ""

    ssh "$V2N_USER@$V2N_IP" "cd ~/ros2_ws/src/bowling_target_nav && python3 test/test_arduino.py --standalone"

    echo ""
    read -p "Press Enter to continue..."
}

run_mock_gui() {
    print_header "Arduino GUI (Mock Mode)"

    info "Starting GUI in mock mode (no hardware needed)..."
    echo ""

    python3 "$PKG_DIR/test/test_arduino.py" --gui --mock
}

run_unit_tests() {
    print_header "Running Unit Tests"

    info "Running all unit tests with mock hardware..."
    echo ""

    cd "$PKG_DIR"
    python3 -m pytest test/unit/ -v --tb=short

    echo ""
    read -p "Press Enter to continue..."
}

run_ssh_test() {
    local test_type="$1"
    print_header "Running $test_type Test on V2N"

    if ! check_v2n_connection; then
        err "Cannot reach V2N."
        read -p "Press Enter to continue..."
        return
    fi

    info "Running $test_type test on V2N..."
    echo ""

    case "$test_type" in
        arduino)
            ssh "$V2N_USER@$V2N_IP" "cd ~/ros2_ws/src/bowling_target_nav && python3 test/test_arduino.py --standalone"
            ;;
        camera)
            ssh "$V2N_USER@$V2N_IP" "cd ~/ros2_ws/src/bowling_target_nav && python3 test/test_camera.py --standalone"
            ;;
        lidar)
            ssh "$V2N_USER@$V2N_IP" "cd ~/ros2_ws/src/bowling_target_nav && python3 test/test_lidar.py --standalone"
            ;;
    esac

    echo ""
    read -p "Press Enter to continue..."
}

run_detection_test() {
    print_header "Detection System Test"

    info "Running detector tests (mock mode)..."
    echo ""

    cd "$PKG_DIR"
    python3 -m pytest test/unit/test_detectors.py -v --tb=short

    echo ""
    read -p "Press Enter to continue..."
}

check_connection_diag() {
    print_header "Connection Diagnostics"

    echo "V2N IP: $V2N_IP"
    echo ""

    check_v2n_connection
    echo ""

    if ping -c 1 -W 2 "$V2N_IP" &> /dev/null; then
        check_ssh_connection
        echo ""

        echo -n "Checking Arduino port ($ARDUINO_PORT)... "
        if nc -z -w2 "$V2N_IP" "$ARDUINO_PORT" 2>/dev/null; then
            echo -e "${GREEN}Open (server running)${NC}"
        else
            echo -e "${YELLOW}Closed (server not running)${NC}"
        fi
    fi

    echo ""
    read -p "Press Enter to continue..."
}

check_ros2_topics() {
    print_header "ROS2 Topics on V2N"

    if ! check_v2n_connection; then
        err "Cannot reach V2N."
        read -p "Press Enter to continue..."
        return
    fi

    info "Listing ROS2 topics on V2N..."
    echo ""

    ssh "$V2N_USER@$V2N_IP" "source /opt/ros/humble/setup.bash && ros2 topic list" 2>/dev/null || \
        warn "Could not list topics. Is ROS2 running on V2N?"

    echo ""
    read -p "Press Enter to continue..."
}

check_hardware_status() {
    print_header "Hardware Status on V2N"

    if ! check_v2n_connection; then
        err "Cannot reach V2N."
        read -p "Press Enter to continue..."
        return
    fi

    echo "Checking hardware on V2N..."
    echo ""

    ssh "$V2N_USER@$V2N_IP" << 'REMOTE_EOF'
echo "=== Serial Ports (Arduino) ==="
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No serial ports found"
echo ""

echo "=== Video Devices (Camera) ==="
ls -la /dev/video* 2>/dev/null || echo "No video devices found"
v4l2-ctl --list-devices 2>/dev/null || true
echo ""

echo "=== USB Devices ==="
lsusb 2>/dev/null | head -10
echo ""

echo "=== DRP Status ==="
ls -la /dev/drp* 2>/dev/null || echo "No DRP devices found"
REMOTE_EOF

    echo ""
    read -p "Press Enter to continue..."
}

view_v2n_info() {
    print_header "V2N System Information"

    if ! check_v2n_connection; then
        err "Cannot reach V2N."
        read -p "Press Enter to continue..."
        return
    fi

    ssh "$V2N_USER@$V2N_IP" << 'REMOTE_EOF'
echo "=== System ==="
uname -a
echo ""

echo "=== Memory ==="
free -h
echo ""

echo "=== Disk ==="
df -h / | tail -1
echo ""

echo "=== CPU Temperature ==="
cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | awk '{print $1/1000 "°C"}' || echo "N/A"
echo ""

echo "=== Network ==="
ip addr show | grep -E "inet |wlan|eth" | head -5
REMOTE_EOF

    echo ""
    read -p "Press Enter to continue..."
}

check_local_ros2() {
    print_header "Local ROS2 Installation"

    ROS2_PATH=$(find_ros2)
    if [ -n "$ROS2_PATH" ]; then
        ok "ROS2 found: $ROS2_PATH"
        source "$ROS2_PATH"
        echo ""
        echo "ROS2 Distribution: $ROS_DISTRO"
        echo "RMW Implementation: ${RMW_IMPLEMENTATION:-default}"
        echo ""

        echo "Checking Python packages..."
        python3 -c "import rclpy; print('  rclpy: OK')" 2>/dev/null || echo "  rclpy: NOT FOUND"
        python3 -c "import tkinter; print('  tkinter: OK')" 2>/dev/null || echo "  tkinter: NOT FOUND"
        python3 -c "import gi; print('  GTK (gi): OK')" 2>/dev/null || echo "  GTK (gi): NOT FOUND"
        python3 -c "import numpy; print('  numpy: OK')" 2>/dev/null || echo "  numpy: NOT FOUND"
    else
        err "ROS2 not found on this system"
        echo ""
        echo "Install ROS2 Humble with:"
        echo "  sudo apt install ros-humble-desktop"
    fi

    echo ""
    read -p "Press Enter to continue..."
}

configure_v2n_ip() {
    print_header "Configure V2N IP Address"

    echo "Current V2N IP: $V2N_IP"
    echo ""
    read -p "Enter new V2N IP (or press Enter to keep current): " new_ip

    if [ -n "$new_ip" ]; then
        V2N_IP="$new_ip"
        ok "V2N IP set to: $V2N_IP"

        # Test connection
        echo ""
        check_v2n_connection
    fi

    echo ""
    read -p "Press Enter to continue..."
}

ssh_to_v2n() {
    print_header "SSH to V2N"

    info "Connecting to $V2N_USER@$V2N_IP..."
    info "Type 'exit' to return to this menu"
    echo ""

    ssh "$V2N_USER@$V2N_IP"
}

create_standalone_controller() {
    cat > "$SCRIPT_DIR/pc_robot_controller_standalone.py" << 'STANDALONE_EOF'
#!/usr/bin/env python3
"""Standalone Robot Controller - No ROS2 Required"""
import socket
import time
import threading
import json
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

class RobotController:
    def __init__(self, host="192.168.50.1", port=9090):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.on_data_callback = None
        self.running = False

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(0.1)
            self.connected = True
            self.running = True
            threading.Thread(target=self._read_loop, daemon=True).start()
            return True, f"Connected to {self.host}:{self.port}"
        except Exception as e:
            return False, str(e)

    def disconnect(self):
        self.running = False
        self.connected = False
        if self.socket:
            try: self.socket.close()
            except: pass
            self.socket = None

    def _read_loop(self):
        buffer = ""
        while self.running and self.socket:
            try:
                data = self.socket.recv(1024).decode('utf-8', errors='ignore')
                if data:
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip() and self.on_data_callback:
                            self.on_data_callback(line.strip())
            except socket.timeout: pass
            except: break

    def send_velocity(self, lx, ly, az):
        if self.connected and self.socket:
            try:
                msg = json.dumps({"op": "publish", "topic": "/cmd_vel",
                    "msg": {"linear": {"x": lx, "y": ly, "z": 0}, "angular": {"x": 0, "y": 0, "z": az}}})
                self.socket.send(f"{msg}\n".encode())
                return True
            except: pass
        return False

class RobotGUI:
    def __init__(self):
        self.robot = RobotController()
        self.robot.on_data_callback = self.on_response
        self.root = tk.Tk()
        self.root.title("V2N Robot Controller (Standalone)")
        self.root.geometry("500x400")
        self._create_widgets()
        self._bind_keys()

    def _create_widgets(self):
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill='both', expand=True)

        conn = ttk.LabelFrame(main, text="Connection", padding=10)
        conn.pack(fill='x', pady=5)
        ttk.Label(conn, text="V2N: 192.168.50.1:9090").pack(side='left')
        self.connect_btn = ttk.Button(conn, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side='left', padx=20)
        self.status = ttk.Label(conn, text="Disconnected", foreground='red')
        self.status.pack(side='left')

        speed = ttk.LabelFrame(main, text="Speed", padding=5)
        speed.pack(fill='x', pady=5)
        ttk.Label(speed, text="Linear:").pack(side='left')
        self.linear_var = tk.DoubleVar(value=0.3)
        ttk.Scale(speed, from_=0.1, to=1.0, variable=self.linear_var, length=80).pack(side='left')
        ttk.Label(speed, text="Angular:").pack(side='left', padx=(10,0))
        self.angular_var = tk.DoubleVar(value=0.5)
        ttk.Scale(speed, from_=0.1, to=2.0, variable=self.angular_var, length=80).pack(side='left')

        ctrl = ttk.LabelFrame(main, text="Controls (WASD/QE)", padding=10)
        ctrl.pack(fill='x', pady=5)
        bf = ttk.Frame(ctrl)
        bf.pack()
        ttk.Button(bf, text="↑", command=self.fwd, width=5).grid(row=0, column=1)
        ttk.Button(bf, text="←", command=self.left, width=5).grid(row=1, column=0)
        ttk.Button(bf, text="■", command=self.stop, width=5).grid(row=1, column=1)
        ttk.Button(bf, text="→", command=self.right, width=5).grid(row=1, column=2)
        ttk.Button(bf, text="↓", command=self.bwd, width=5).grid(row=2, column=1)
        ttk.Button(bf, text="↺", command=self.rot_l, width=5).grid(row=1, column=3, padx=10)
        ttk.Button(bf, text="↻", command=self.rot_r, width=5).grid(row=1, column=4)

        self.log = scrolledtext.ScrolledText(main, height=6, state='disabled')
        self.log.pack(fill='both', expand=True, pady=5)

    def _bind_keys(self):
        for k in ['w','W']: self.root.bind(f'<{k}>', lambda e: self.fwd()); self.root.bind(f'<KeyRelease-{k}>', lambda e: self.stop())
        for k in ['s','S']: self.root.bind(f'<{k}>', lambda e: self.bwd()); self.root.bind(f'<KeyRelease-{k}>', lambda e: self.stop())
        for k in ['a','A']: self.root.bind(f'<{k}>', lambda e: self.left()); self.root.bind(f'<KeyRelease-{k}>', lambda e: self.stop())
        for k in ['d','D']: self.root.bind(f'<{k}>', lambda e: self.right()); self.root.bind(f'<KeyRelease-{k}>', lambda e: self.stop())
        self.root.bind('<q>', lambda e: self.rot_l()); self.root.bind('<e>', lambda e: self.rot_r())
        self.root.bind('<space>', lambda e: self.stop())

    def toggle_connection(self):
        if self.robot.connected:
            self.robot.disconnect()
            self.connect_btn.config(text="Connect")
            self.status.config(text="Disconnected", foreground='red')
        else:
            ok, msg = self.robot.connect()
            if ok:
                self.connect_btn.config(text="Disconnect")
                self.status.config(text="Connected", foreground='green')
                self._log(msg)
            else:
                messagebox.showerror("Error", f"Failed: {msg}\n\nRun on V2N:\nros2 launch rosbridge_server rosbridge_websocket_launch.xml")

    def on_response(self, data): self.root.after(0, lambda: self._log(f"← {data}"))
    def _log(self, msg):
        self.log.config(state='normal')
        self.log.insert('end', f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.log.see('end')
        self.log.config(state='disabled')

    def fwd(self): self.robot.send_velocity(self.linear_var.get(), 0, 0)
    def bwd(self): self.robot.send_velocity(-self.linear_var.get(), 0, 0)
    def left(self): self.robot.send_velocity(0, self.linear_var.get(), 0)
    def right(self): self.robot.send_velocity(0, -self.linear_var.get(), 0)
    def rot_l(self): self.robot.send_velocity(0, 0, self.angular_var.get())
    def rot_r(self): self.robot.send_velocity(0, 0, -self.angular_var.get())
    def stop(self): self.robot.send_velocity(0, 0, 0)

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", lambda: (self.robot.disconnect(), self.root.destroy()))
        self.root.mainloop()

if __name__ == "__main__":
    RobotGUI().run()
STANDALONE_EOF
    chmod +x "$SCRIPT_DIR/pc_robot_controller_standalone.py"
}

# ============================================================================
# Menu Loops
# ============================================================================

arduino_menu_loop() {
    while true; do
        show_arduino_menu
        read -r choice
        case "$choice" in
            1) start_arduino_server ;;
            2) run_arduino_gui ;;
            3) run_arduino_quick_test ;;
            4) run_mock_gui ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

test_menu_loop() {
    while true; do
        show_test_menu
        read -r choice
        case "$choice" in
            1) run_unit_tests ;;
            2) run_ssh_test "arduino" ;;
            3) run_ssh_test "camera" ;;
            4) run_ssh_test "lidar" ;;
            5) run_detection_test ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

diag_menu_loop() {
    while true; do
        show_diag_menu
        read -r choice
        case "$choice" in
            1) check_connection_diag ;;
            2) check_ros2_topics ;;
            3) check_hardware_status ;;
            4) view_v2n_info ;;
            5) check_local_ros2 ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

main_menu_loop() {
    while true; do
        show_main_menu
        read -r choice
        case "$choice" in
            1) run_ros2_gui ;;
            2) run_standalone_gui ;;
            3) arduino_menu_loop ;;
            4) test_menu_loop ;;
            5) diag_menu_loop ;;
            6) ssh_to_v2n ;;
            c|C) configure_v2n_ip ;;
            q|Q) echo ""; exit 0 ;;
            *) ;;
        esac
    done
}

# ============================================================================
# Main Entry Point
# ============================================================================

# Handle command line arguments
case "${1:-}" in
    gui)
        run_ros2_gui
        ;;
    standalone)
        run_standalone_gui
        ;;
    arduino)
        arduino_menu_loop
        ;;
    test)
        test_menu_loop
        ;;
    diag)
        diag_menu_loop
        ;;
    help|--help|-h)
        echo "V2N Robot Controller - All-in-One Tool"
        echo ""
        echo "Usage: $0 [command]"
        echo ""
        echo "Commands:"
        echo "  (none)      Show interactive menu"
        echo "  gui         Run ROS2 robot controller GUI"
        echo "  standalone  Run standalone controller (no ROS2)"
        echo "  arduino     Arduino remote control menu"
        echo "  test        Hardware tests menu"
        echo "  diag        System diagnostics menu"
        echo ""
        echo "Environment variables:"
        echo "  V2N_IP      V2N robot IP address (default: 192.168.50.1)"
        echo "  V2N_USER    SSH user for V2N (default: root)"
        echo ""
        ;;
    *)
        main_menu_loop
        ;;
esac
