# V2N Setup Guide — From Scratch

Complete guide to deploy target_nav on a fresh RZ/V2N board.

---

## Prerequisites

**Hardware connected to V2N:**

| Port | Device | Purpose |
|------|--------|---------|
| USB → auto-detected (`ttyUSB*`/`ttyACM*`) | Arduino Mega | Motor control + encoder feedback (driver finds it by firmware handshake) |
| USB → `/dev/ttyUSB0` | RPLidar A1 | 2D LiDAR scanner |
| USB → `/dev/video0` | Camera | RGB video for DRP-AI detection |
| DSI-1 | Touchscreen | GUI display (rotated 180°) |
| wlan0 | WiFi module | WiFi Access Point |

**On the V2N board (pre-installed with Renesas BSP):**
- ROS2 Humble at `/opt/ros/humble`
- Python 3.12+

**On your PC:**
- SSH access to V2N
- The target_nav project folder

---

## Step 1: Connect to V2N

Connect V2N to your PC via Ethernet. Default IP is `192.168.50.1`.

```bash
ssh root@192.168.50.1
```

If this is a brand new board, you may need to set a static IP first.

---

## Step 2: Copy Project to V2N

From your **PC** (not the V2N):

```bash
# Create workspace on V2N
ssh root@192.168.50.1 "mkdir -p /root/ros2_ws/src/target_nav /root/deploy"

# Copy the Python package + ROS2 files
scp -r target_nav/ setup.py setup.cfg package.xml README.md \
       root@192.168.50.1:/root/ros2_ws/src/target_nav/

# Copy launch, config, URDF, resource, scripts, test
scp -r launch/ config/ urdf/ resource/ scripts/ test/ \
       root@192.168.50.1:/root/ros2_ws/src/target_nav/

# Copy DRP-AI binary and model
scp -r deploy/* root@192.168.50.1:/home/root/deploy/
```

---

## Step 3: Run Full Setup

SSH into the V2N and run the one-shot provisioning script:

```bash
ssh root@192.168.50.1
cd /root/ros2_ws/src/target_nav/scripts/setup
chmod +x v2n_setup.sh
./v2n_setup.sh
```

This script does **everything** automatically:

1. Installs Python dependencies (`pyserial`)
2. Builds the ROS2 package with `colcon build`
3. Deploys DRP-AI binary and libraries to `/usr/lib/`
4. Installs helper scripts (`gui.sh`, `launcher.py`, `remote_desktop.py`) to `/root/`
5. Sets up WiFi Access Point (SSID: `RZV2N_Robot`, password: `robot1234`)
6. Configures display rotation for DSI touchscreen
7. Creates and enables 5 systemd services
8. Checks hardware connections

**Options:**
```bash
./v2n_setup.sh              # Full setup
./v2n_setup.sh --no-build   # Skip colcon build (if already built)
./v2n_setup.sh --wifi-only  # Only set up WiFi AP
./v2n_setup.sh --status     # Check service status
```

---

## Step 4: Reboot

```bash
reboot
```

On boot, the system automatically starts:

| Service | What it does |
|---------|-------------|
| `seatd` | Display manager (must start first) |
| `wifi-ap` | WiFi AP at 192.168.50.1 (SSID: RZV2N_Robot) |
| `robot` | Bringup (LiDAR, Arduino, Odometry) |
| `target-nav-launcher` | Start/Stop GUI button on touchscreen |
| `remote-desktop` | Web remote at http://192.168.50.1:8080 |

---

## Step 5: Connect and Use

### From the touchscreen
Tap **"Start"** on the launcher to open the GUI.

### From your PC (WiFi)
1. Connect to WiFi: **RZV2N_Robot** (password: **robot1234**)
2. Open browser: **http://192.168.50.1:8080**
3. You'll see the V2N screen remotely
4. Click **Start** to launch the GUI

### From your PC (SSH)
```bash
ssh root@192.168.50.1
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Check ROS2 topics
ros2 topic list

# Check what's running
ros2 node list

# View LiDAR data
ros2 topic echo /scan --once

# View robot pose
ros2 topic echo /odom --once
```

---

## What's Running After Boot

```
┌─────────────────────────────────────────────────┐
│ Infrastructure (launched by bringup.launch.py)   │
│   arduino_driver_node  → motor control (serial)  │
│   odometry_node        → encoder → odom TF       │
│   robot_state_publisher→ URDF → TF tree          │
│   rplidar_node         → LiDAR → /scan           │
└─────────────────────────────────────────────────┘

When GUI starts (user clicks "Start"):

┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│ Core 0: GUI      │  │ Core 1: nav_node │  │ Core 2: camera   │
│                  │  │  (nice -5)       │  │                  │
│ GTK3 window      │  │ Navigator        │  │ DRP-AI C++ binary│
│ Map + camera     │  │ Obstacle avoid   │  │ YOLO detection   │
│ Settings panel   │  │ Blind approach   │  │ (Core 3)         │
│                  │  │ Search scan      │  │                  │
│ ←─ NavShmReader  │  │── /cmd_vel ────→ │  │─ DetShmWriter ──→│
│ ←─ LaserShmReader│  │←─ DetShmReader   │  │ /dev/shm frames  │
│ ─→ CmdRingBuffer │  │←─ CmdRingBuffer  │  │                  │
└──────────────────┘  └──────────────────┘  └──────────────────┘

Note: GUI has NO ROS2. All GUI data flows through struct SHM.
```

---

## Daily Development Workflow

After the initial setup, use `sync.sh` to push code changes:

```bash
# From your PC, in the project directory:
./scripts/sync.sh
```

This automatically:
1. Tests SSH connection
2. Copies all files to V2N
3. Cleans build directories
4. Runs `colcon build`

Then restart the GUI on the V2N (tap Stop → Start, or):
```bash
ssh root@192.168.50.1 "systemctl restart robot"
```

---

## Network Setup

```
PC (192.168.50.xxx)  ←──WiFi──→  V2N (192.168.50.1)
                                     │
                                     ├── WiFi AP: RZV2N_Robot
                                     │   Password: robot1234
                                     │   DHCP: 192.168.50.100-200
                                     │
                                     ├── SSH: port 22
                                     ├── Remote Desktop: port 8080
                                     └── ROS2 DDS: domain 0
```

ROS2 topics are accessible from your PC over WiFi:
```bash
# On your PC (after connecting to RZV2N_Robot WiFi):
export ROS_DOMAIN_ID=0
ros2 topic list                    # See V2N topics
ros2 topic echo /detections        # See live detections
```

---

## File Locations on V2N

| Path | Content |
|------|---------|
| `/root/ros2_ws/src/target_nav/` | Source code |
| `/root/ros2_ws/install/target_nav/` | Built package |
| `/root/deploy/` | DRP-AI binary + model |
| `/root/gui.sh` | Symlink to GUI launcher |
| `/root/start.sh` | Symlink to bringup script |
| `~/.config/target_nav/calibration.json` | Saved settings |
| `/var/log/robot_autostart.log` | Boot log |
| `/dev/shm/v2n_camera` | Live camera frame (C++ → GUI, mmap) |
| `/dev/shm/v2n_detections` | C++ detection structs (C++ → Camera process) |
| `/dev/shm/v2n_det` | Python detection structs (Camera → Nav + GUI) |
| `/dev/shm/v2n_nav` | Nav state snapshot (Nav → GUI) |
| `/dev/shm/v2n_laser` | LiDAR points + pose (Nav → GUI) |
| `/dev/shm/v2n_cmd` | Command ring buffer (GUI → Nav) |
| `/dev/shm/v2n_calibration` | Calibration params (Camera → C++) |

---

## Troubleshooting

### Check service status
```bash
systemctl status robot
systemctl status wifi-ap
systemctl status target-nav-launcher
```

### View logs
```bash
# Robot autostart log
cat /var/log/robot_autostart.log

# Systemd journal (live)
journalctl -u robot -f

# Last 50 lines
journalctl -u robot -n 50
```

### Hardware not detected
```bash
# Check USB devices (Arduino + LiDAR enumerate as ttyUSB*/ttyACM*; numbers vary)
ls -la /dev/ttyUSB* /dev/ttyACM* /dev/video0

# If missing, reconnect USB and check dmesg:
dmesg | tail -20
```

### Rebuild after code changes
```bash
cd /root/ros2_ws
rm -rf build/target_nav install/target_nav
source /opt/ros/humble/setup.bash
colcon build --packages-select target_nav --symlink-install
```

### Stop everything
```bash
systemctl stop robot target-nav-launcher remote-desktop
pkill -f "ros2 launch"
pkill -f main_gui
pkill -f app_yolo_cam
```

### Reset calibration to defaults
```bash
rm ~/.config/target_nav/calibration.json
# Restart GUI — defaults from config.py will be used
```

### WiFi AP not working
```bash
systemctl restart wifi-ap
ip addr show wlan0
# Should show 192.168.50.1
```

---

## Quick Reference

| Action | Command |
|--------|---------|
| SSH in | `ssh root@192.168.50.1` |
| Start GUI | Tap "Start" on screen, or `ssh` → `/root/gui.sh` |
| Stop everything | `systemctl stop robot` |
| Restart robot | `systemctl restart robot` |
| View logs | `journalctl -u robot -f` |
| Sync code from PC | `./scripts/sync.sh` |
| Check topics | `ros2 topic list` |
| Manual bringup | `ros2 launch target_nav bringup.launch.py` |
| Check services | `systemctl status robot wifi-ap target-nav-launcher` |
