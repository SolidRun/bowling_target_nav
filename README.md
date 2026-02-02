# Bowling Target Navigation - RZ/V2N

SLAM mapping + Camera with YOLO detection + PC Remote Control for RZ/V2N robot.

**Auto-start enabled**: Robot starts automatically on boot with GUI, SLAM, and camera.

## Quick Start

### On V2N (Robot)

The robot **starts automatically on boot** with:
- GUI showing camera feed + SLAM map
- All ROS2 nodes ready for PC connection
- Arduino, LiDAR, and camera initialized

To manually control:
```bash
ssh root@192.168.50.1
systemctl status robot    # Check status
systemctl restart robot   # Restart if needed
```

### On PC (Remote Control)

```bash
# 1. Connect to V2N WiFi network
# 2. Run the controller
cd ~/ros2_ws/src/bowling_target_nav/tools
./run_controller.sh
```

## Installation on New V2N

### Quick One-Liner (Recommended)

```bash
# From your PC - copies package and sets up with auto-start
scp -r bowling_target_nav root@192.168.50.1:~/ros2_ws/src/ && \
ssh root@192.168.50.1 "cd ~/ros2_ws/src/bowling_target_nav/scripts && chmod +x *.sh && ./v2n_setup.sh --autostart"
```

### Manual Setup

```bash
# Step 1: Copy package to V2N
scp -r bowling_target_nav root@192.168.50.1:~/ros2_ws/src/

# Step 2: SSH and run setup
ssh root@192.168.50.1
cd ~/ros2_ws/src/bowling_target_nav/scripts
chmod +x *.sh
./v2n_setup.sh           # Without auto-start
# OR
./v2n_setup.sh --autostart  # With auto-start on boot
```

### Using Tar Archive (Faster Transfer)

```bash
# Create and copy archive
cd /path/to/ros2_ws/src
tar czf bowling_nav.tar.gz bowling_target_nav
scp bowling_nav.tar.gz root@192.168.50.1:/tmp/

# Extract and setup on V2N
ssh root@192.168.50.1 "cd ~/ros2_ws/src && tar xzf /tmp/bowling_nav.tar.gz && cd bowling_target_nav/scripts && chmod +x *.sh && ./v2n_setup.sh --autostart"
```

### What v2n_setup.sh Does

The setup script performs these steps automatically:

1. **Verifies ROS2** - Checks ROS2 Humble is installed
2. **Creates workspace** - Creates ~/ros2_ws if it doesn't exist
3. **Makes scripts executable** - chmod +x on all .sh and .py files
4. **Builds package** - colcon build with symlink-install
5. **Checks hardware** - Verifies Arduino, LiDAR, Camera connections
6. **Verifies ROS2 registration** - Confirms package is registered
7. **Installs autostart** (with --autostart flag) - Enables boot service

After setup, the V2N will automatically start all services on boot.

## Hardware Requirements

- **Arduino**: Motor controller on `/dev/ttyACM0` (Mecanum 4WD)
- **LiDAR**: RPLidar A1 on `/dev/ttyUSB0`
- **Camera**: USB camera on `/dev/video0` (optional)
- **Display**: Wayland/Weston (HDMI) for GUI

## Commands

### Start Robot (V2N)

```bash
# Basic - just motor control (for PC remote control)
~/ros2_ws/src/bowling_target_nav/scripts/start_robot.sh

# Full - with SLAM mapping
~/ros2_ws/src/bowling_target_nav/scripts/start_robot.sh --full

# Stop all
~/ros2_ws/src/bowling_target_nav/scripts/start_robot.sh --stop

# GUI with SLAM + Camera (on V2N display)
~/ros2_ws/src/bowling_target_nav/scripts/setup_and_run.sh
```

### PC Controller

```bash
# Auto-setup ROS2 and run GUI
cd tools/
./run_controller.sh
```

**Controls:**
- **WASD** or **Arrow Keys**: Move robot
- **Q/E**: Rotate left/right
- **Space**: Emergency stop
- **Release key**: Stops movement
- **Sliders**: Adjust speed
- **Calibrate**: Send SYNC command to Arduino
- **Read/Reset Encoders**: Encoder commands
- **Manual**: Send custom Arduino commands

## Auto-Start on Boot

The robot automatically starts all services when V2N powers on (~20 seconds boot time).

**What starts automatically:**
- Arduino driver (motor control)
- LiDAR driver (RPLidar)
- Odometry node
- SLAM mapping (Cartographer)
- GUI with camera + YOLO detection

**Install autostart (if not already enabled):**
```bash
ssh root@192.168.50.1
cd ~/ros2_ws/src/bowling_target_nav/scripts
./install_autostart.sh
```

**Service Commands:**
```bash
systemctl status robot     # Check status
systemctl restart robot    # Restart
systemctl stop robot       # Stop
journalctl -u robot -f     # View logs
./install_autostart.sh --remove  # Disable autostart
```

**ROS2 Topics Available:**
- `/cmd_vel` - Velocity commands (for PC controller)
- `/arduino/cmd` - Direct Arduino commands (SYNC, READ, RESET)
- `/arduino/status` - Arduino connection status
- `/scan` - LiDAR data
- `/odom` - Odometry
- `/map` - SLAM map

## Package Contents

```
bowling_target_nav/
├── bowling_target_nav/
│   ├── nodes/
│   │   ├── slam_camera_gui.py    # V2N GUI (SLAM + Camera + YOLO)
│   │   ├── arduino_driver_node.py # Motor control via Arduino
│   │   └── odometry_node.py       # Wheel odometry
│   ├── hardware/
│   │   └── arduino_bridge.py      # Arduino serial communication
│   ├── detectors/
│   │   └── yolo_detector.py       # YOLO detection
│   └── utils/
│       └── distance_estimator.py  # Distance estimation
├── config/
│   ├── cartographer.lua           # SLAM configuration
│   └── target_nav_params.yaml
├── launch/
│   ├── bringup.launch.py          # LiDAR + Arduino + Odometry
│   └── mapping.launch.py          # Cartographer SLAM
├── scripts/
│   ├── start_robot.sh             # Start robot for remote control
│   ├── setup_and_run.sh           # Full GUI setup
│   ├── run_slam_camera.sh         # SLAM + Camera GUI
│   ├── v2n_setup.sh               # New V2N installation
│   ├── robot_autostart.sh         # Auto-start script (systemd)
│   ├── robot.service              # Systemd service file
│   └── install_autostart.sh       # Install/remove autostart
├── tools/
│   ├── pc_robot_controller.py     # PC GUI controller
│   └── run_controller.sh          # PC setup + run script
└── models/
    └── bowling_yolov5.onnx        # YOLO model (optional)
```

## Troubleshooting

### Robot Not Responding to Commands

```bash
# Check if Arduino is connected
ls -la /dev/ttyACM0

# Kill processes holding the port
fuser -k /dev/ttyACM0

# Restart robot
~/ros2_ws/src/bowling_target_nav/scripts/start_robot.sh --stop
~/ros2_ws/src/bowling_target_nav/scripts/start_robot.sh
```

### PC Controller Not Connecting

```bash
# Verify V2N is reachable
ping 192.168.50.1

# Check ROS2 topics on V2N
ssh root@192.168.50.1 "source /opt/ros/humble/setup.bash && ros2 topic list"

# Should see:
#   /cmd_vel
#   /arduino/status
#   /scan
#   /odom
```

### LiDAR Timeout Error

```bash
# Release LiDAR port
fuser -k /dev/ttyUSB0

# Check device
ls -la /dev/ttyUSB0
```

### Camera Busy

```bash
fuser -k /dev/video0
```

### GUI Not Starting on Boot

```bash
# Check service status
systemctl status robot

# View service logs
journalctl -u robot -n 50

# Manually start
systemctl start robot

# If service file is missing, reinstall
cd ~/ros2_ws/src/bowling_target_nav/scripts
./install_autostart.sh
```

### Map Not Showing in GUI

The map requires TRANSIENT_LOCAL QoS. If "Waiting for map..." appears:
```bash
# Restart the service
systemctl restart robot

# Verify map topic
ros2 topic echo /map --once
```

### Rebuild Package

```bash
cd ~/ros2_ws
rm -rf build/bowling_target_nav install/bowling_target_nav
colcon build --packages-select bowling_target_nav --symlink-install
source install/setup.bash
```

## Network Configuration

V2N default IP: `192.168.50.1` (WiFi hotspot)

Connect to V2N WiFi network, then:
```bash
ssh root@192.168.50.1
```

## Features

- **Auto-Start on Boot**: Robot ready in ~20 seconds after power on
- **Mecanum Drive**: Full omnidirectional control (forward, strafe, rotate)
- **SLAM Mapping**: Real-time 2D map using Cartographer + RPLidar
- **YOLO Detection**: Bowling pin detection with distance estimation
- **PC Remote Control**: Control robot via WiFi from your PC
- **GUI on V2N**: Fullscreen display with camera feed + SLAM map
- **Arduino Commands**: Calibration, encoder read/reset from PC GUI
- **Arduino Protocol**: Plain text commands (FWD, BWD, LEFT, RIGHT, TURN, STOP, SYNC)
- **Fast Startup**: Optimized boot sequence (~20 seconds)
- **Thread-safe**: Proper locking, timeout protection, clean shutdown

## Technical Notes

**ROS2 Communication:**
- Uses `rmw_fastrtps_cpp` middleware for PC-V2N communication
- FastDDS XML config for WiFi peer discovery (192.168.50.1)
- ROS_DOMAIN_ID=0 for cross-machine communication

**Camera:**
- Uses V4L2 backend directly (cv2.CAP_V4L2) - GStreamer has issues on V2N

**Map Subscription:**
- Uses TRANSIENT_LOCAL QoS to match Cartographer's map publisher

**Arduino Protocol:**
- Plain text commands: `CMD,speed,ticks\n` (e.g., `FWD,100,1000`)
- Wait for "READY" message after serial connect (Arduino resets on connect)
