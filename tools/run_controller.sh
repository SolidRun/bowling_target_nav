#!/bin/bash
# ============================================================================
# V2N Robot Controller - Setup and Run
# ============================================================================
#
# This script:
# 1. Finds and sources ROS2 (Humble, Jazzy, or Rolling)
# 2. Installs missing Python dependencies
# 3. Runs the robot controller GUI
#
# Usage: ./run_controller.sh
# ============================================================================

set -e

echo "=============================================="
echo "  V2N Robot Controller (ROS2)"
echo "=============================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ============================================================================
# Step 1: Check V2N connectivity
# ============================================================================
echo "Checking V2N connectivity..."
if ping -c 1 -W 2 192.168.50.1 &> /dev/null; then
    echo -e "${GREEN}[OK]${NC} V2N robot is reachable at 192.168.50.1"
else
    echo -e "${YELLOW}[WARNING]${NC} Cannot reach 192.168.50.1"
    echo "    Make sure you're connected to V2N WiFi network"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi
echo ""

# ============================================================================
# Step 2: Find and source ROS2
# ============================================================================
echo "Looking for ROS2 installation..."

ROS2_FOUND=false
ROS2_DISTRO=""

# Check common ROS2 locations
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
        echo -e "${GREEN}[OK]${NC} Found ROS2 at: $ros_path"
        source "$ros_path"
        ROS2_FOUND=true
        ROS2_DISTRO=$(echo "$ros_path" | grep -oP '(?<=/ros/)[^/]+' || echo "custom")
        break
    fi
done

if [ "$ROS2_FOUND" = false ]; then
    echo -e "${RED}[ERROR]${NC} ROS2 not found!"
    echo ""
    echo "ROS2 is required to run this controller."
    echo ""
    echo "Install options:"
    echo "  1. Install ROS2 Humble (Ubuntu 22.04):"
    echo "     sudo apt install software-properties-common"
    echo "     sudo add-apt-repository universe"
    echo "     sudo apt update && sudo apt install curl -y"
    echo "     sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
    echo "     echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main\" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"
    echo "     sudo apt update"
    echo "     sudo apt install ros-humble-desktop python3-colcon-common-extensions"
    echo ""
    echo "  2. Or use the standalone version (no ROS2 required):"
    echo "     python3 $SCRIPT_DIR/pc_robot_controller_standalone.py"
    echo ""

    read -p "Would you like to create a standalone version? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Creating standalone controller..."
        python3 "$SCRIPT_DIR/create_standalone.py" 2>/dev/null || {
            echo "Creating standalone version..."
            # Create inline
            cat > "$SCRIPT_DIR/pc_robot_controller_standalone.py" << 'STANDALONE_EOF'
#!/usr/bin/env python3
"""
Standalone Robot Controller - No ROS2 Required
Uses direct socket connection to V2N
"""

import socket
import time
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

class RobotController:
    """Direct socket connection to V2N robot"""

    def __init__(self, host="192.168.50.1", port=9090):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.on_data_callback = None
        self.running = False
        self.read_thread = None

    def connect(self):
        """Connect to robot"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(0.1)
            self.connected = True
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            return True, f"Connected to {self.host}:{self.port}"
        except Exception as e:
            return False, str(e)

    def disconnect(self):
        """Disconnect from robot"""
        self.running = False
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None

    def _read_loop(self):
        """Read responses"""
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
            except socket.timeout:
                pass
            except Exception as e:
                if self.running:
                    print(f"Read error: {e}")
                break

    def send(self, cmd):
        """Send command"""
        if self.connected and self.socket:
            try:
                self.socket.send(f"{cmd}\n".encode())
                return True
            except Exception as e:
                print(f"Send error: {e}")
        return False

    def send_velocity(self, linear_x, linear_y, angular_z):
        """Send velocity command as JSON"""
        import json
        msg = json.dumps({
            "op": "publish",
            "topic": "/cmd_vel",
            "msg": {
                "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
            }
        })
        return self.send(msg)


class RobotGUI:
    """Standalone GUI"""

    def __init__(self):
        self.robot = RobotController()
        self.robot.on_data_callback = self.on_response
        self.linear_speed = 0.3
        self.angular_speed = 0.5

        self.root = tk.Tk()
        self.root.title("V2N Robot Controller (Standalone)")
        self.root.geometry("600x500")

        self._create_widgets()
        self._bind_keys()

    def _create_widgets(self):
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill='both', expand=True)

        # Connection
        conn = ttk.LabelFrame(main, text="Connection", padding=10)
        conn.pack(fill='x', pady=5)

        ttk.Label(conn, text="Robot: 192.168.50.1:9090").pack(side='left')
        self.connect_btn = ttk.Button(conn, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side='left', padx=20)
        self.status_label = ttk.Label(conn, text="Disconnected", foreground='red')
        self.status_label.pack(side='left')

        # Note
        ttk.Label(main, text="Note: Requires rosbridge_server on V2N", foreground='gray').pack()

        # Speed
        speed = ttk.LabelFrame(main, text="Speed", padding=10)
        speed.pack(fill='x', pady=5)

        ttk.Label(speed, text="Linear:").pack(side='left')
        self.linear_var = tk.DoubleVar(value=0.3)
        ttk.Scale(speed, from_=0.1, to=1.0, variable=self.linear_var, length=100).pack(side='left')

        ttk.Label(speed, text="Angular:").pack(side='left', padx=(20,0))
        self.angular_var = tk.DoubleVar(value=0.5)
        ttk.Scale(speed, from_=0.1, to=2.0, variable=self.angular_var, length=100).pack(side='left')

        # Controls
        ctrl = ttk.LabelFrame(main, text="Controls (WASD)", padding=10)
        ctrl.pack(fill='x', pady=5)

        btn_frame = ttk.Frame(ctrl)
        btn_frame.pack()

        ttk.Button(btn_frame, text="↑ FWD", command=self.move_forward, width=10).grid(row=0, column=1, pady=2)
        ttk.Button(btn_frame, text="← LEFT", command=self.move_left, width=10).grid(row=1, column=0, padx=2)
        ttk.Button(btn_frame, text="STOP", command=self.stop, width=10).grid(row=1, column=1)
        ttk.Button(btn_frame, text="→ RIGHT", command=self.move_right, width=10).grid(row=1, column=2, padx=2)
        ttk.Button(btn_frame, text="↓ BWD", command=self.move_backward, width=10).grid(row=2, column=1, pady=2)

        rot = ttk.Frame(ctrl)
        rot.pack(pady=10)
        ttk.Button(rot, text="↺ Rotate L", command=self.rotate_left, width=12).pack(side='left', padx=10)
        ttk.Button(rot, text="↻ Rotate R", command=self.rotate_right, width=12).pack(side='left')

        # Log
        log = ttk.LabelFrame(main, text="Log", padding=5)
        log.pack(fill='both', expand=True, pady=5)
        self.log_text = scrolledtext.ScrolledText(log, height=8, state='disabled')
        self.log_text.pack(fill='both', expand=True)

    def _bind_keys(self):
        for key in ['w', 'W', 'Up']:
            self.root.bind(f'<{key}>', lambda e: self.move_forward())
            self.root.bind(f'<KeyRelease-{key}>', lambda e: self.stop())
        for key in ['s', 'S', 'Down']:
            self.root.bind(f'<{key}>', lambda e: self.move_backward())
            self.root.bind(f'<KeyRelease-{key}>', lambda e: self.stop())
        for key in ['a', 'A', 'Left']:
            self.root.bind(f'<{key}>', lambda e: self.move_left())
            self.root.bind(f'<KeyRelease-{key}>', lambda e: self.stop())
        for key in ['d', 'D', 'Right']:
            self.root.bind(f'<{key}>', lambda e: self.move_right())
            self.root.bind(f'<KeyRelease-{key}>', lambda e: self.stop())
        self.root.bind('<space>', lambda e: self.stop())
        self.root.bind('<q>', lambda e: self.rotate_left())
        self.root.bind('<e>', lambda e: self.rotate_right())

    def toggle_connection(self):
        if self.robot.connected:
            self.robot.disconnect()
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Disconnected", foreground='red')
            self.log("Disconnected")
        else:
            success, msg = self.robot.connect()
            if success:
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text="Connected", foreground='green')
                self.log(f"Connected: {msg}")
            else:
                messagebox.showerror("Error", f"Connection failed:\n{msg}\n\nMake sure rosbridge_server is running on V2N")
                self.log(f"Error: {msg}")

    def on_response(self, data):
        self.root.after(0, lambda: self.log(f"← {data}"))

    def log(self, msg):
        self.log_text.config(state='normal')
        self.log_text.insert('end', f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.log_text.see('end')
        self.log_text.config(state='disabled')

    def move_forward(self):
        if self.robot.connected:
            self.robot.send_velocity(self.linear_var.get(), 0, 0)
            self.log("→ FWD")

    def move_backward(self):
        if self.robot.connected:
            self.robot.send_velocity(-self.linear_var.get(), 0, 0)
            self.log("→ BWD")

    def move_left(self):
        if self.robot.connected:
            self.robot.send_velocity(0, self.linear_var.get(), 0)
            self.log("→ LEFT")

    def move_right(self):
        if self.robot.connected:
            self.robot.send_velocity(0, -self.linear_var.get(), 0)
            self.log("→ RIGHT")

    def rotate_left(self):
        if self.robot.connected:
            self.robot.send_velocity(0, 0, self.angular_var.get())
            self.log("→ ROTATE L")

    def rotate_right(self):
        if self.robot.connected:
            self.robot.send_velocity(0, 0, -self.angular_var.get())
            self.log("→ ROTATE R")

    def stop(self):
        if self.robot.connected:
            self.robot.send_velocity(0, 0, 0)
            self.log("→ STOP")

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def on_close(self):
        self.robot.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    print("V2N Robot Controller (Standalone)")
    print("Requires rosbridge_server on V2N")
    app = RobotGUI()
    app.run()
STANDALONE_EOF
            chmod +x "$SCRIPT_DIR/pc_robot_controller_standalone.py"
            echo -e "${GREEN}[OK]${NC} Created standalone controller"
        }
        echo ""
        echo "Run with: python3 $SCRIPT_DIR/pc_robot_controller_standalone.py"
        echo ""
        echo "Note: The standalone version requires rosbridge_server on V2N:"
        echo "  ssh root@192.168.50.1"
        echo "  ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
        exit 0
    fi
    exit 1
fi

echo -e "${GREEN}[OK]${NC} ROS2 $ROS2_DISTRO sourced"

# ============================================================================
# Step 3: Set ROS2 environment for WiFi communication
# ============================================================================
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Use FastRTPS to match V2N (MUST use same RMW implementation)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Configure FastDDS for unicast discovery over WiFi
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_wifi.xml

# Create FastDDS config for WiFi peer discovery
cat > /tmp/fastdds_wifi.xml << 'FASTDDS_EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <ignoreParticipantFlags>FILTER_DIFFERENT_PROCESS</ignoreParticipantFlags>
                </discovery_config>
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4>
                            <address>0.0.0.0</address>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>192.168.50.1</address>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
FASTDDS_EOF

echo -e "${GREEN}[OK]${NC} ROS_DOMAIN_ID=0"
echo -e "${GREEN}[OK]${NC} FastRTPS configured with peer: 192.168.50.1 (matches V2N)"

# ============================================================================
# Step 4: Check Python dependencies
# ============================================================================
echo ""
echo "Checking Python dependencies..."

MISSING_DEPS=()

# Check tkinter
python3 -c "import tkinter" 2>/dev/null || MISSING_DEPS+=("python3-tk")

# Check rclpy
python3 -c "import rclpy" 2>/dev/null || MISSING_DEPS+=("ros-${ROS2_DISTRO}-rclpy")

if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    echo -e "${YELLOW}[WARNING]${NC} Missing dependencies: ${MISSING_DEPS[*]}"
    echo ""
    read -p "Install missing dependencies? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt update
        sudo apt install -y "${MISSING_DEPS[@]}"
    else
        echo "Some features may not work without dependencies."
    fi
else
    echo -e "${GREEN}[OK]${NC} All dependencies installed"
fi

# ============================================================================
# Step 5: Run the controller
# ============================================================================
echo ""
echo "=============================================="
echo "Starting V2N Robot Controller..."
echo "=============================================="
echo ""

python3 "$SCRIPT_DIR/pc_robot_controller.py"
