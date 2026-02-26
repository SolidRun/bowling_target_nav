# Bowling Target Navigation — RZ/V2N Robot

> [!WARNING]
> This project is under active development. APIs, configuration, and behavior may change without notice. Use at your own risk in production environments.

> Autonomous bowling pin detection and navigation for the Renesas RZ/V2N mecanum robot.
> Combines SLAM mapping, YOLO AI detection, and holonomic navigation in a single fullscreen GUI.

```
┌─────────────────────────────────────────────────────────────────┐
│                     V2N Robot Control GUI                        │
│  ┌────────────────────────┐  ┌────────────────────────────────┐ │
│  │                        │  │                                │ │
│  │     SLAM Map           │  │    Camera + YOLO Detection     │ │
│  │   (robot, laser,       │  │  (bounding boxes, distance,    │ │
│  │    grid, target)       │  │   angle, crosshair)            │ │
│  │                        │  │                                │ │
│  └────────────────────────┘  └────────────────────────────────┘ │
│  [GO TO TARGET]  [STOP]   ● NAVIGATING 0.45m  [SETTINGS] [QUIT]│
└─────────────────────────────────────────────────────────────────┘
```

---

## Documentation

| Guide | Description |
|-------|-------------|
| [Architecture](docs/ARCHITECTURE.md) | System architecture, block diagrams, design patterns |
| [How It Works](docs/HOW_IT_WORKS.md) | Complete technical deep-dive: sensing, AI detection, navigation |
| [GUI Guide](docs/GUI_GUIDE.md) | GUI layout, controls, settings, keyboard shortcuts |
| [Hardware Setup](docs/HARDWARE_SETUP.md) | Hardware wiring, Arduino protocol, sensor configuration |
| [API Reference](docs/API_REFERENCE.md) | Module, class, and function reference for developers |

---

## Quick Start

### Prerequisites

| Component | Requirement |
|-----------|-------------|
| Platform | Renesas RZ/V2N board (Yocto Linux) |
| ROS2 | Humble Hawksbill |
| Arduino | Motor controller on `/dev/ttyACM0` (115200 baud) |
| LiDAR | RPLidar A1 on `/dev/ttyUSB0` |
| Camera | USB camera on `/dev/video0` (640x480) |
| Display | HDMI (Wayland/Weston) |
| Network | WiFi AP at `192.168.50.1` |

### First-Time Setup (One Command)

```bash
# SSH into V2N
ssh root@192.168.50.1

# Copy package (from PC)
scp -r bowling_target_nav root@192.168.50.1:~/ros2_ws/src/

# Run setup — builds, installs services, starts GUI
cd ~/ros2_ws/src/bowling_target_nav/scripts
./v2n_setup.sh
```

This single command:
1. Verifies ROS2 Humble is installed
2. Builds the package with `colcon build`
3. Installs a systemd service for auto-start on boot
4. Starts the fullscreen GUI with SLAM + Camera + Navigation

### Daily Usage

After setup, the robot **starts automatically on boot**. Manual control:

```bash
./v2n_setup.sh --start    # Start GUI
./v2n_setup.sh --stop     # Stop everything
./v2n_setup.sh --status   # Check status
./v2n_setup.sh --build    # Rebuild package
```

### Service Commands

```bash
# Robot hardware (LiDAR, motors, SLAM, odometry)
systemctl status robot       # Check status
systemctl restart robot      # Restart
journalctl -u robot -f       # Live logs

# GUI launcher (Start/Stop GUI floating button)
systemctl status bowling-launcher
systemctl restart bowling-launcher

# Remote desktop (web-based at http://192.168.50.1:8080)
systemctl status remote-desktop
systemctl restart remote-desktop
```

---

## How It Works (Summary)

```
Camera (30fps)              LiDAR (10Hz)            Wheel Encoders (20Hz)
     │                          │                         │
     ▼                          ▼                         ▼
YOLO Detection         Cartographer SLAM           Odometry Node
(DRP-AI Stream/Pipe    (2D occupancy grid)        (odom → base_link TF)
 or ONNX CPU)
     │                          │                         │
     └──────────┬───────────────┘                         │
                ▼                                         │
        Navigation Engine (20Hz)  ◄────────────────────────┘
        ├── Vision+LiDAR fusion
        ├── Holonomic path planning
        ├── VFH obstacle avoidance
        ├── 360° odometry-tracked search scan
        ├── Blind approach (dead-reckoning)
        └── Multi-signal arrival detection
                │
                ▼
          /cmd_vel (Twist)
                │
                ▼
        Arduino Motor Controller
        (VEL,vx,vy,wz → 4 mecanum wheels)
```

The system runs **three threads**:
1. **GTK Main Thread** (30fps) — Renders GUI, handles user input
2. **ROS2 Thread** (20Hz) — SLAM/LiDAR callbacks, navigation control loop
3. **Camera Thread** (~30fps) — Frame capture, async YOLO detection

All threads communicate through a **thread-safe shared state** with RLock and 0.1s timeouts.

---

## GUI Controls

| Control | Action |
|---------|--------|
| **GO** button / `G` key | Start navigating to detected bowling pin |
| **STOP** button / `Space` / `S` | Emergency stop |
| **SETTINGS** button | Open 5-tab parameter tuning window |
| **QUIT** button / `Q` / `ESC` | Quit application |

The GUI shows:
- **Left panel**: SLAM map with robot position (green), LiDAR points (red), navigation target (magenta)
- **Right panel**: Camera feed with YOLO bounding boxes, crosshair on closest pin, distance/angle labels
- **Status bar**: Color-coded navigation state with speed and obstacle info

---

## Navigation States

```
            ┌────────── STOP pressed ──────────────────┐
            │                                          │
            ▼         GO pressed                       │
      ┌──► IDLE ◄──────────── 360° scan complete        │
      │     │                                          │
      │     │ No target visible                        │
      │     ▼                                          │
      │  SEARCHING ──── Target found! ──────┐          │
      │     ▲                               │          │
      │     │                               ▼          │
      │  Target lost >3s           NAVIGATING ◄────────┤
      │  AND far (>0.8m)               │               │
      │     │                          │               │
      │     │                 Target lost >3s           │
      │     │                 AND close (<0.8m)         │
      │     │                          │               │
      │     │                          ▼               │
      │     └─────────────  BLIND_APPROACH             │
      │                          │                     │
      │                 Arrived / LiDAR stop            │
      │                          │                     │
      │                          ▼                     │
      └───────────────────  ARRIVED ───────────────────┘
                         (terminal)
```

---

## Detection Backends

| Backend | Description | Performance | Config Value |
|---------|-------------|-------------|--------------|
| **DRP-AI Stream** | C++ owns camera + inference, Python reads via shared memory | ~5-10ms inference, zero-copy frames | `drp_stream` |
| **DRP-AI Pipe** | Python sends frames to C++ subprocess via stdin/stdout | ~10-20ms inference | `drp_binary` |
| **YOLO ONNX** | CPU inference (fallback) | ~45ms inference | `yolo_onnx` |
| **Mock** | Testing without camera | Instant | `mock` |

```bash
# Switch backend via environment variable
export V2N_DETECTOR_TYPE=drp_binary

# Or edit config/robot_config.yaml
```

The system **auto-detects**: tries DRP-AI first (V2N hardware), falls back to ONNX CPU.

---

## ROS2 Topics

### Published

| Topic | Type | Source | Purpose |
|-------|------|--------|---------|
| `/cmd_vel` | Twist | Navigation engine | Motor velocity commands |
| `/odom` | Odometry | odometry_node | Wheel encoder odometry |
| `/target_pose` | PoseStamped | vision_node | Detected pin position |
| `/target_detection` | String | vision_node | Detection JSON metadata |
| `/arduino/status` | String | arduino_driver | Connection status |
| `/arduino/odom_raw` | String | arduino_driver | Raw encoder telemetry |
| `/diagnostics` | DiagnosticArray | arduino_driver | System health |

### Subscribed

| Topic | Type | Consumer | Purpose |
|-------|------|----------|---------|
| `/cmd_vel` | Twist | arduino_driver | Motor control |
| `/scan` | LaserScan | Navigation, SLAM | LiDAR obstacle data |
| `/map` | OccupancyGrid | GUI, navigation | SLAM map |
| `/target_pose` | PoseStamped | target_follower | Pin position |
| `/gui/command` | String | MainGuiNode | GUI command dispatch |

### TF Frames

```
map ──► odom ──► base_link ──► laser
   (SLAM)  (encoders)     ├──► camera_link
                          └──► wheel_fl/fr/rl/rr
```

---

## PC Remote Control

Control the robot wirelessly from your PC:

```bash
# Connect to V2N WiFi, then:
cd tools/
python3 pc_robot_controller.py
```

**Controls**: WASD movement, Q/E rotation, Space stop, speed sliders.

---

## Project Structure

```
bowling_target_nav/
├── bowling_target_nav/           # Main Python package
│   ├── nodes/                    # 9 ROS2 entry points
│   │   └── main_gui.py          #   Primary: 3-thread GUI application
│   ├── state/                    # Thread-safe shared state (Singleton + Facade)
│   │   ├── shared_state.py      #   Composes 3 domain stores
│   │   ├── sensor_store.py      #   Map, pose, laser
│   │   ├── detection_store.py   #   Camera, detections, tunable params
│   │   └── nav_store.py         #   Nav state, commands, obstacles
│   ├── nav/                      # Navigation algorithms
│   │   ├── navigator.py         #   Holonomic drive, blind approach, obstacles
│   │   └── target_selector.py   #   Closest pin selection
│   ├── threads/                  # Worker threads
│   │   ├── ros_node.py          #   ROS2 node + 20Hz control loop
│   │   └── camera_worker.py     #   Camera + async YOLO detection
│   ├── gui/                      # GTK3 interface
│   │   ├── main_window.py       #   Fullscreen window
│   │   ├── settings_window.py   #   5-tab parameter tuning
│   │   └── panels/              #   Map and camera renderers
│   ├── detectors/                # AI backends (Strategy pattern)
│   │   ├── base.py              #   DetectorBase ABC
│   │   ├── yolo_onnx_detector.py#   ONNX Runtime CPU
│   │   └── drp_binary_detector.py#  DRP-AI hardware
│   ├── hardware/                 # Hardware abstractions (Factory pattern)
│   │   ├── arduino.py           #   Motor control + encoders
│   │   ├── camera.py            #   Camera capture
│   │   └── lidar.py             #   LiDAR bridge
│   └── utils/
│       └── distance_estimator.py #  Bbox size → distance
├── config/                       # YAML + Lua configuration
├── launch/                       # 7 ROS2 launch files
├── models/                       # YOLO ONNX models
├── scripts/                      # Setup and service scripts
├── tools/                        # PC remote control
├── test/                         # Unit + integration tests
├── urdf/                         # Robot description (TF frames)
└── docs/                         # Documentation
```

---

## Testing

```bash
cd scripts/
./run_tests.sh              # All tests
./run_tests.sh arduino      # Arduino motor tests
./run_tests.sh lidar        # LiDAR sensor tests
./run_tests.sh camera       # Camera + YOLO tests
./run_tests.sh integration  # Sensor fusion tests
./run_tests.sh system       # Full system tests
./run_tests.sh --check      # Check hardware availability
```

### Interactive Test GUIs

```bash
./run_tests.sh --gui                # Full system control
./run_tests.sh --visualize-lidar    # LiDAR visualization
./run_tests.sh --visualize-camera   # Camera detection demo
./run_tests.sh --visualize-fusion   # Sensor fusion view
```

---

## Network

| Host | IP | Access |
|------|-----|--------|
| V2N Robot | `192.168.50.1` | `ssh root@192.168.50.1` |

---

## Troubleshooting

### Hardware Check

```bash
ls -la /dev/ttyACM0 /dev/ttyUSB0 /dev/video0
```

### Robot Not Responding

```bash
./v2n_setup.sh --stop && ./v2n_setup.sh --start
```

### GUI Not Starting

```bash
journalctl -u robot -n 50    # Check logs
echo $WAYLAND_DISPLAY        # Should be "wayland-0"
echo $XDG_RUNTIME_DIR        # Should be "/run"
```

### Rebuild

```bash
./v2n_setup.sh --build
```

---

## License

MIT
