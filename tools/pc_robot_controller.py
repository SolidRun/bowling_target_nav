#!/usr/bin/env python3
"""
PC Robot Controller - ROS2 GUI for V2N Robot Control
=====================================================

Sends cmd_vel commands to the robot over WiFi via ROS2.
Shows Arduino connection status from V2N.

Requirements:
    - ROS2 (Humble/Jazzy/Rolling) installed on PC
    - Connected to V2N WiFi network (192.168.50.1)
    - V2N running bringup.launch.py

Usage:
    ./run_controller.sh    # Recommended - sets up ROS2 environment
    # Or manually:
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=0
    python3 pc_robot_controller.py
"""

import sys
import time
import json
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from typing import Optional

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ERROR: ROS2 not available. Please source your ROS2 setup.bash")
    print("  source /opt/ros/humble/setup.bash  # or jazzy/rolling")
    sys.exit(1)


class RobotControllerNode(Node):
    """ROS2 node for sending commands to the robot"""

    def __init__(self):
        super().__init__('pc_robot_controller')

        # QoS for cmd_vel (best effort for real-time)
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # QoS for status (reliable, latched)
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', cmd_vel_qos)

        # Publisher for direct Arduino commands (calibration, read encoders, etc.)
        self.arduino_cmd_pub = self.create_publisher(String, '/arduino/cmd', 10)

        # Subscriber for Arduino status
        self.arduino_status_sub = self.create_subscription(
            String, '/arduino/status', self.on_arduino_status, status_qos)

        # Subscriber for Arduino raw odometry
        self.arduino_odom_sub = self.create_subscription(
            String, '/arduino/odom_raw', self.on_arduino_odom, 10)

        # Callbacks
        self.status_callback = None
        self.odom_callback = None

        # Arduino state
        self.arduino_connected = False
        self.arduino_port = "Unknown"
        self.last_status_time = 0

        self.get_logger().info('PC Robot Controller node started')

    def on_arduino_status(self, msg):
        """Handle Arduino status messages"""
        try:
            data = json.loads(msg.data)
            self.arduino_connected = data.get('state') == 'connected'
            self.arduino_port = data.get('port', 'Unknown')
            self.last_status_time = time.time()

            if self.status_callback:
                self.status_callback(self.arduino_connected, self.arduino_port)
        except json.JSONDecodeError:
            pass

    def on_arduino_odom(self, msg):
        """Handle Arduino odometry messages"""
        if self.odom_callback:
            self.odom_callback(msg.data)

    def send_velocity(self, linear_x: float, linear_y: float, angular_z: float):
        """Send velocity command"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        """Emergency stop"""
        self.send_velocity(0.0, 0.0, 0.0)

    def send_arduino_cmd(self, command: str):
        """Send direct command to Arduino via ROS2"""
        msg = String()
        msg.data = command
        self.arduino_cmd_pub.publish(msg)


class RobotGUI:
    """Main GUI application for controlling the robot via ROS2"""

    def __init__(self):
        self.node: Optional[RobotControllerNode] = None
        self.ros_thread: Optional[threading.Thread] = None
        self.ros_running = False
        self.connected = False

        # Speed settings
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s

        # Create main window
        self.root = tk.Tk()
        self.root.title("V2N Robot Controller (ROS2)")
        self.root.geometry("750x700")
        self.root.resizable(True, True)

        # Configure style
        self._configure_style()
        self._create_widgets()
        self._bind_keys()

        # Start periodic update
        self._update_status()

    def _configure_style(self):
        """Configure ttk styles"""
        style = ttk.Style()

        available_themes = style.theme_names()
        for theme in ['clam', 'alt', 'default']:
            if theme in available_themes:
                style.theme_use(theme)
                break

        style.configure('Connected.TLabel', foreground='green', font=('Arial', 10, 'bold'))
        style.configure('Disconnected.TLabel', foreground='red', font=('Arial', 10, 'bold'))
        style.configure('Warning.TLabel', foreground='orange', font=('Arial', 10, 'bold'))
        style.configure('Title.TLabel', font=('Arial', 12, 'bold'))

    def _create_widgets(self):
        """Create GUI widgets"""
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill='both', expand=True)

        # === Connection Frame ===
        conn_frame = ttk.LabelFrame(main_frame, text="ROS2 Connection", padding=10)
        conn_frame.pack(fill='x', pady=(0, 10))

        conn_row = ttk.Frame(conn_frame)
        conn_row.pack(fill='x')

        ttk.Label(conn_row, text="V2N Robot: 192.168.50.1").pack(side='left', padx=5)

        self.connect_btn = ttk.Button(conn_row, text="Connect", command=self.toggle_connection, width=12)
        self.connect_btn.pack(side='left', padx=20)

        self.status_label = ttk.Label(conn_row, text="Disconnected", style='Disconnected.TLabel')
        self.status_label.pack(side='left', padx=10)

        # Arduino status row
        arduino_row = ttk.Frame(conn_frame)
        arduino_row.pack(fill='x', pady=(10, 0))

        ttk.Label(arduino_row, text="Arduino:").pack(side='left', padx=5)
        self.arduino_status_label = ttk.Label(arduino_row, text="Unknown", style='Warning.TLabel')
        self.arduino_status_label.pack(side='left', padx=5)

        self.arduino_port_label = ttk.Label(arduino_row, text="", foreground='gray')
        self.arduino_port_label.pack(side='left', padx=10)

        # Info label
        info_frame = ttk.Frame(conn_frame)
        info_frame.pack(fill='x', pady=(5, 0))
        ttk.Label(info_frame, text="Make sure V2N is running: ros2 launch bowling_target_nav bringup.launch.py",
                  foreground='gray', font=('Arial', 8)).pack(side='left')

        # === Speed Settings ===
        speed_frame = ttk.LabelFrame(main_frame, text="Speed Settings", padding=10)
        speed_frame.pack(fill='x', pady=(0, 10))

        speed_row = ttk.Frame(speed_frame)
        speed_row.pack(fill='x')

        ttk.Label(speed_row, text="Linear (m/s):").pack(side='left', padx=(0, 5))
        self.linear_var = tk.DoubleVar(value=0.2)
        ttk.Scale(speed_row, from_=0.05, to=0.5, variable=self.linear_var,
                  orient='horizontal', length=150).pack(side='left', padx=(0, 5))
        self.linear_label = ttk.Label(speed_row, text="0.20", width=5)
        self.linear_label.pack(side='left', padx=(0, 20))

        ttk.Label(speed_row, text="Angular (rad/s):").pack(side='left', padx=(0, 5))
        self.angular_var = tk.DoubleVar(value=0.5)
        ttk.Scale(speed_row, from_=0.1, to=1.5, variable=self.angular_var,
                  orient='horizontal', length=150).pack(side='left', padx=(0, 5))
        self.angular_label = ttk.Label(speed_row, text="0.50", width=5)
        self.angular_label.pack(side='left')

        # Update labels when sliders change
        self.linear_var.trace('w', lambda *_: self.linear_label.config(text=f"{self.linear_var.get():.2f}"))
        self.angular_var.trace('w', lambda *_: self.angular_label.config(text=f"{self.angular_var.get():.2f}"))

        # === Movement Controls ===
        ctrl_frame = ttk.LabelFrame(main_frame, text="Movement Controls (WASD + QE)", padding=10)
        ctrl_frame.pack(fill='x', pady=(0, 10))

        # Direction buttons grid
        dir_frame = ttk.Frame(ctrl_frame)
        dir_frame.pack(pady=10)

        buttons = [
            (0, 0, "FL", lambda: self.move_diagonal(1, 1)),
            (0, 1, "FWD\n[W]", lambda: self.move_forward()),
            (0, 2, "FR", lambda: self.move_diagonal(1, -1)),
            (1, 0, "LEFT\n[A]", lambda: self.move_left()),
            (1, 1, "STOP\n[Space]", lambda: self.stop()),
            (1, 2, "RIGHT\n[D]", lambda: self.move_right()),
            (2, 0, "BL", lambda: self.move_diagonal(-1, 1)),
            (2, 1, "BWD\n[S]", lambda: self.move_backward()),
            (2, 2, "BR", lambda: self.move_diagonal(-1, -1)),
        ]

        for row, col, text, cmd in buttons:
            btn = ttk.Button(dir_frame, text=text, width=10, command=cmd)
            btn.grid(row=row, column=col, padx=4, pady=4, sticky='nsew')

        # Rotation buttons
        rotate_frame = ttk.Frame(ctrl_frame)
        rotate_frame.pack(pady=10)

        ttk.Button(rotate_frame, text="Rotate Left [Q]", width=16,
                   command=self.rotate_left).pack(side='left', padx=20)
        ttk.Button(rotate_frame, text="Rotate Right [E]", width=16,
                   command=self.rotate_right).pack(side='left', padx=20)

        # === Quick Commands ===
        quick_frame = ttk.LabelFrame(main_frame, text="Quick Commands", padding=10)
        quick_frame.pack(fill='x', pady=(0, 10))

        quick_row1 = ttk.Frame(quick_frame)
        quick_row1.pack(fill='x', pady=(0, 5))

        ttk.Button(quick_row1, text="STOP", command=self.stop, width=10).pack(side='left', padx=3)
        ttk.Button(quick_row1, text="FWD 1m", command=lambda: self.timed_move(0.2, 0, 0, 5), width=10).pack(side='left', padx=3)
        ttk.Button(quick_row1, text="BWD 1m", command=lambda: self.timed_move(-0.2, 0, 0, 5), width=10).pack(side='left', padx=3)
        ttk.Button(quick_row1, text="LEFT 1m", command=lambda: self.timed_move(0, 0.2, 0, 5), width=10).pack(side='left', padx=3)
        ttk.Button(quick_row1, text="RIGHT 1m", command=lambda: self.timed_move(0, -0.2, 0, 5), width=10).pack(side='left', padx=3)
        ttk.Button(quick_row1, text="Turn 90L", command=lambda: self.timed_move(0, 0, 0.5, 3.14), width=10).pack(side='left', padx=3)
        ttk.Button(quick_row1, text="Turn 90R", command=lambda: self.timed_move(0, 0, -0.5, 3.14), width=10).pack(side='left', padx=3)

        # Arduino commands row
        quick_row2 = ttk.Frame(quick_frame)
        quick_row2.pack(fill='x')

        ttk.Label(quick_row2, text="Arduino:").pack(side='left', padx=(0, 5))
        ttk.Button(quick_row2, text="Calibrate", command=self.calibrate, width=10).pack(side='left', padx=3)
        ttk.Button(quick_row2, text="Read Encoders", command=self.read_encoders, width=12).pack(side='left', padx=3)
        ttk.Button(quick_row2, text="Reset Encoders", command=self.reset_encoders, width=12).pack(side='left', padx=3)

        # Manual command entry
        ttk.Label(quick_row2, text="Manual:").pack(side='left', padx=(15, 5))
        self.manual_cmd_var = tk.StringVar()
        manual_entry = ttk.Entry(quick_row2, textvariable=self.manual_cmd_var, width=15)
        manual_entry.pack(side='left', padx=3)
        manual_entry.bind('<Return>', lambda e: self.send_manual_cmd())
        ttk.Button(quick_row2, text="Send", command=self.send_manual_cmd, width=6).pack(side='left', padx=3)

        # === Log Frame ===
        log_frame = ttk.LabelFrame(main_frame, text="Log", padding=10)
        log_frame.pack(fill='both', expand=True, pady=(0, 10))

        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, state='disabled',
                                                   font=('Consolas', 9), wrap='word')
        self.log_text.pack(fill='both', expand=True)

        self.log_text.tag_config('tx', foreground='#0066cc')
        self.log_text.tag_config('rx', foreground='#009933')
        self.log_text.tag_config('error', foreground='#cc0000')
        self.log_text.tag_config('info', foreground='#666666')
        self.log_text.tag_config('status', foreground='#996600')

        # Log controls
        log_ctrl = ttk.Frame(log_frame)
        log_ctrl.pack(fill='x', pady=(5, 0))
        ttk.Button(log_ctrl, text="Clear Log", command=self.clear_log, width=10).pack(side='right')

        # === Status Bar ===
        status_bar = ttk.Frame(main_frame)
        status_bar.pack(fill='x')
        ttk.Label(status_bar, text="Keys: W/A/S/D=Move | Q/E=Rotate | Space=Stop | Release key to stop",
                  font=('Arial', 8), foreground='gray').pack()

    def _bind_keys(self):
        """Bind keyboard shortcuts"""
        # WASD movement
        self.root.bind('<w>', lambda e: self.move_forward())
        self.root.bind('<W>', lambda e: self.move_forward())
        self.root.bind('<s>', lambda e: self.move_backward())
        self.root.bind('<S>', lambda e: self.move_backward())
        self.root.bind('<a>', lambda e: self.move_left())
        self.root.bind('<A>', lambda e: self.move_left())
        self.root.bind('<d>', lambda e: self.move_right())
        self.root.bind('<D>', lambda e: self.move_right())

        # Arrow keys
        self.root.bind('<Up>', lambda e: self.move_forward())
        self.root.bind('<Down>', lambda e: self.move_backward())
        self.root.bind('<Left>', lambda e: self.move_left())
        self.root.bind('<Right>', lambda e: self.move_right())

        # Rotation
        self.root.bind('<q>', lambda e: self.rotate_left())
        self.root.bind('<Q>', lambda e: self.rotate_left())
        self.root.bind('<e>', lambda e: self.rotate_right())
        self.root.bind('<E>', lambda e: self.rotate_right())

        # Stop
        self.root.bind('<space>', lambda e: self.stop())
        self.root.bind('<Escape>', lambda e: self.stop())

        # Key release - stop movement
        for key in ['w', 'W', 's', 'S', 'a', 'A', 'd', 'D', 'q', 'Q', 'e', 'E', 'Up', 'Down', 'Left', 'Right']:
            self.root.bind(f'<KeyRelease-{key}>', lambda e: self.stop())

    def _update_status(self):
        """Periodic status update"""
        # Update speed values
        self.linear_speed = self.linear_var.get()
        self.angular_speed = self.angular_var.get()

        # Check Arduino status timeout
        if self.connected and self.node:
            if time.time() - self.node.last_status_time > 5:
                self.arduino_status_label.config(text="No Status", style='Warning.TLabel')

        self.root.after(100, self._update_status)

    def on_arduino_status(self, connected: bool, port: str):
        """Handle Arduino status update"""
        def update():
            if connected:
                self.arduino_status_label.config(text="Connected", style='Connected.TLabel')
                self.arduino_port_label.config(text=f"({port})")
            else:
                self.arduino_status_label.config(text="Disconnected", style='Disconnected.TLabel')
                self.arduino_port_label.config(text="")
        self.root.after(0, update)

    def toggle_connection(self):
        """Connect or disconnect from ROS2"""
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        """Start ROS2 connection"""
        try:
            # Initialize ROS2
            if not rclpy.ok():
                rclpy.init()

            # Create node
            self.node = RobotControllerNode()
            self.node.status_callback = self.on_arduino_status

            # Start ROS2 spin thread
            self.ros_running = True
            self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
            self.ros_thread.start()

            self.connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Connected", style='Connected.TLabel')
            self.log("ROS2 connected - publishing to /cmd_vel", 'info')
            self.log("Waiting for Arduino status from V2N...", 'status')

        except Exception as e:
            self.log(f"Connection error: {e}", 'error')
            messagebox.showerror("Error", f"Failed to connect:\n{e}")

    def disconnect(self):
        """Stop ROS2 connection"""
        # Send stop command before disconnecting
        if self.node:
            self.node.stop()

        self.ros_running = False

        if self.node:
            self.node.destroy_node()
            self.node = None

        if self.ros_thread:
            self.ros_thread.join(timeout=1)

        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Disconnected", style='Disconnected.TLabel')
        self.arduino_status_label.config(text="Unknown", style='Warning.TLabel')
        self.arduino_port_label.config(text="")
        self.log("Disconnected", 'info')

    def _ros_spin(self):
        """ROS2 spin loop in background thread"""
        while self.ros_running and self.node:
            try:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            except Exception as e:
                if self.ros_running:
                    print(f"ROS spin error: {e}")
                break

    def log(self, msg: str, tag: str = None):
        """Add message to log"""
        self.log_text.config(state='normal')
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] {msg}\n"

        if tag:
            self.log_text.insert('end', line, tag)
        else:
            self.log_text.insert('end', line)

        self.log_text.see('end')
        self.log_text.config(state='disabled')

    def clear_log(self):
        """Clear log"""
        self.log_text.config(state='normal')
        self.log_text.delete('1.0', 'end')
        self.log_text.config(state='disabled')

    # Movement commands
    def move_forward(self):
        """Move forward"""
        if self.node and self.connected:
            self.node.send_velocity(self.linear_speed, 0.0, 0.0)
            self.log(f"FWD @ {self.linear_speed:.2f} m/s", 'tx')

    def move_backward(self):
        """Move backward"""
        if self.node and self.connected:
            self.node.send_velocity(-self.linear_speed, 0.0, 0.0)
            self.log(f"BWD @ {self.linear_speed:.2f} m/s", 'tx')

    def move_left(self):
        """Strafe left (mecanum)"""
        if self.node and self.connected:
            self.node.send_velocity(0.0, self.linear_speed, 0.0)
            self.log(f"LEFT @ {self.linear_speed:.2f} m/s", 'tx')

    def move_right(self):
        """Strafe right (mecanum)"""
        if self.node and self.connected:
            self.node.send_velocity(0.0, -self.linear_speed, 0.0)
            self.log(f"RIGHT @ {self.linear_speed:.2f} m/s", 'tx')

    def move_diagonal(self, x_dir: int, y_dir: int):
        """Diagonal movement (mecanum)"""
        if self.node and self.connected:
            speed = self.linear_speed * 0.707  # Normalize diagonal
            self.node.send_velocity(speed * x_dir, speed * y_dir, 0.0)
            self.log(f"DIAG ({x_dir}, {y_dir}) @ {speed:.2f} m/s", 'tx')

    def rotate_left(self):
        """Rotate left"""
        if self.node and self.connected:
            self.node.send_velocity(0.0, 0.0, self.angular_speed)
            self.log(f"ROTATE LEFT @ {self.angular_speed:.2f} rad/s", 'tx')

    def rotate_right(self):
        """Rotate right"""
        if self.node and self.connected:
            self.node.send_velocity(0.0, 0.0, -self.angular_speed)
            self.log(f"ROTATE RIGHT @ {self.angular_speed:.2f} rad/s", 'tx')

    def stop(self):
        """Stop all movement"""
        if self.node and self.connected:
            self.node.stop()

    def timed_move(self, vx: float, vy: float, wz: float, duration: float):
        """Move for a specific duration then stop"""
        if not self.node or not self.connected:
            self.log("Not connected", 'error')
            return

        self.log(f"Timed move: vx={vx}, vy={vy}, wz={wz} for {duration}s", 'tx')
        self.node.send_velocity(vx, vy, wz)

        # Schedule stop after duration
        def delayed_stop():
            if self.node and self.connected:
                self.node.stop()
                self.log("Timed move complete", 'info')

        self.root.after(int(duration * 1000), delayed_stop)

    def calibrate(self):
        """Send calibration/sync command to Arduino"""
        if not self.node or not self.connected:
            self.log("Not connected", 'error')
            return
        self.node.send_arduino_cmd("SYNC")
        self.log("Sent SYNC (calibration) command", 'tx')

    def read_encoders(self):
        """Read encoder values from Arduino"""
        if not self.node or not self.connected:
            self.log("Not connected", 'error')
            return
        self.node.send_arduino_cmd("READ")
        self.log("Sent READ command (check V2N terminal for response)", 'tx')

    def reset_encoders(self):
        """Reset encoder counters on Arduino"""
        if not self.node or not self.connected:
            self.log("Not connected", 'error')
            return
        self.node.send_arduino_cmd("RESET")
        self.log("Sent RESET command", 'tx')

    def send_manual_cmd(self):
        """Send manual command to Arduino"""
        if not self.node or not self.connected:
            self.log("Not connected", 'error')
            return
        cmd = self.manual_cmd_var.get().strip()
        if not cmd:
            return
        self.node.send_arduino_cmd(cmd)
        self.log(f"Sent manual command: {cmd}", 'tx')
        self.manual_cmd_var.set("")  # Clear entry

    def run(self):
        """Start GUI"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def on_close(self):
        """Handle window close"""
        self.disconnect()
        try:
            rclpy.shutdown()
        except:
            pass
        self.root.destroy()


def main():
    print("=" * 50)
    print("V2N Robot Controller (ROS2)")
    print("=" * 50)
    print("Connect to V2N WiFi network first!")
    print("Robot IP: 192.168.50.1")
    print("=" * 50)

    app = RobotGUI()
    app.run()


if __name__ == "__main__":
    main()
