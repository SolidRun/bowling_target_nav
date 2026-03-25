# Target Navigation — RZ/V2N Robot

> **Disclaimer:** This project is under active development and is **not intended for production use**. It may contain bugs, untested edge cases, or incomplete features. Documentation may not always be up to date with the latest code changes. If you choose to use, modify, or deploy this software, it is **your responsibility** to review the code, verify the documentation, and test thoroughly on your own hardware before relying on it. Use at your own risk.

Autonomous bowling-pin detection and navigation for the Renesas RZ/V2N mecanum robot. DRP-AI hardware-accelerated YOLOv8 detection, holonomic VFH obstacle avoidance with odometry-only localization, all in a fullscreen GTK3 GUI.

---

## Quick Links

| I want to... | Read... |
|--------------|---------|
| Understand the system | [ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| Understand the navigation algorithms | [NAVIGATION.md](docs/NAVIGATION.md) |
| Deploy to V2N from scratch | [V2N_SETUP_GUIDE.md](docs/V2N_SETUP_GUIDE.md) |
| Develop and test on PC | [DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md) |
| Learn in order | [LEARNING_PATHS.md](docs/LEARNING_PATHS.md) |
| Use the GUI | [GUI_GUIDE.md](docs/GUI_GUIDE.md) |
| Wire the hardware | [HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md) |
| Build DRP-AI C++ binary | [drpai/README.md](drpai/README.md) |

---

## Hardware

| Component | Model | Purpose |
|-----------|-------|---------|
| Compute | Renesas RZ/V2N (4x Cortex-A55 + DRP-AI) | Main computer, AI inference |
| Motors | 4x DC with encoders, mecanum wheels | Omnidirectional drive |
| Controller | Arduino Mega + PCA9685 motor shield | Motor PWM, encoder reading |
| LiDAR | RPLidar A1 (360°, 0.15–12m) | Obstacle detection |
| Camera | USB camera (640x480, 30fps) | Target detection via YOLO |
| Display | DSI touchscreen (rotated 180°) | GUI output |

---

## Quick Start

```bash
# First-time setup (on V2N):
ssh root@192.168.50.1
cd /root/ros2_ws/src/target_nav/scripts/setup
./v2n_setup.sh          # Installs everything, creates services, reboots

# Daily use:
# Press "Start" on the touchscreen launcher
# Or from PC: connect to WiFi "RZV2N_Robot" (password: robot1234)
# Open http://192.168.50.1:8080 for remote desktop

# Development cycle (from PC):
./scripts/sync.sh       # Push code + rebuild on V2N
```

---

## Architecture (3-process, lock-free struct SHM)

3 Python processes pinned to CPU cores with lock-free SPSC struct SHM IPC:

| Core | Process | Role |
|------|---------|------|
| 0 | GUI (main) | Pure GTK — reads struct SHM directly, no ROS2 |
| 1 | Nav (nice -5) | ROS2 navigation, Kalman tracker, motor commands |
| 2 | Camera | DRP-AI subprocess management, detection tracking |
| 3 | DRP-AI C++ binary | YOLOv8 inference (pinned to isolate from nav) |

### Data flow

```
Camera (Core 2) ──DetShmWriter──► Nav (Core 1) via DetShmReader [20Hz]
Camera (Core 2) ──DetShmWriter──► GUI (Core 0) via DetShmReader [20Hz]
C++ (Core 3) ──/dev/shm/v2n_camera──► GUI (Core 0) [raw BGR frames, mmap]
Nav (Core 1) ──NavShmWriter──► GUI (Core 0) via NavShmReader [20Hz]
Nav (Core 1) ──LaserShmWriter──► GUI (Core 0) via LaserShmReader [20Hz]
GUI (Core 0) ──CmdRingBuffer──► Nav (Core 1) [GO, STOP, settings]
```

All inter-process communication uses lock-free SPSC struct SHM with torn-read protection (sequence number sandwich). No locks, no JSON for real-time data.

---

## Detection Pipeline

Target: bowling-pin (0.28m height × 0.08m width).

```
DRP-AI C++ Binary (Core 3)
    │
    ├── Camera frames → /dev/shm/v2n_camera (GUI reads directly)
    └── Detections → /dev/shm/v2n_detections (binary structs)
            │
            ▼
    camera_worker.py (Core 2):
    ├── C++ applies: confidence, NMS, size/aspect filters
    ├── Python applies: shape filter (width/height ratio)
    ├── Python applies: temporal tracking (DetectionTracker)
    └── Python pinhole model distance estimation
            │
            ▼
    DetShmWriter → Nav + GUI via DetShmReader [20Hz]
```

The 3-layer filtering pipeline rejects false positives: C++ catches noise (Layer 1), Python shape filter catches wrong proportions like chair legs (Layer 2), Python size-distance gate catches wrong scale (Layer 3). See [NAVIGATION.md § 6](docs/NAVIGATION.md#6-detection-filtering-false-positive-rejection) for details.

---

## Navigation States

The robot navigates through 6 states:

```
IDLE ──GO──► NAVIGATING ──arrived──► ARRIVED
               │    │                    │
      lost+far │    │ lost+close         │ GO
               ▼    ▼                    │
          SEARCHING  BLIND_APPROACH ─────┘
               │
          SPIRAL_SEARCH
```

| State | What the robot does | Key threshold |
|-------|--------------------|----|
| **IDLE** | Stopped, waiting for GO | — |
| **NAVIGATING** | Driving to target with VFH obstacle avoidance | Speed ramp: 0.20→0.10→0.05 m/s |
| **SEARCHING** | 360° rotation scan to re-find lost target | Triggers after 3.0s without detection |
| **SPIRAL_SEARCH** | Expanding Archimedean spiral with camera sweep | After 360° scan fails, up to 2.0m radius |
| **BLIND_APPROACH** | Dead-reckoning via odometry when target lost at close range | Triggers when lost < 0.35m from target |
| **ARRIVED** | Stopped at target, waiting for next GO | Distance ≤ 0.22m for 0.3s |

See [NAVIGATION.md](docs/NAVIGATION.md) for full algorithm details, VFH obstacle avoidance, Kalman filter, arrival detection, and troubleshooting.

---

## Settings (GUI)

5 main tabs (Navigate, Search, Sensors, Radar, Tools) with sub-tabs covering all tunable parameters. All auto-saved to `~/.config/target_nav/calibration.json` with 2s debounce. See [GUI_GUIDE.md § 8](docs/GUI_GUIDE.md#8-settings-window) for full parameter reference with ranges and defaults.

---

## Project Structure

```
bowling_target_nav/
├── target_nav/              # Python package
│   ├── config.py            # Single source of truth for all parameters
│   ├── app/                 # Entry points: main.py, nav_node.py, camera_node.py
│   ├── nav/                 # Navigation: VFH, blind approach, search, arrival
│   ├── detectors/           # DRP-AI detection pipeline
│   ├── hardware/            # Arduino, Camera, LiDAR drivers
│   ├── gui/                 # GTK3 GUI: panels/, settings_tabs/
│   ├── state/               # Thread-safe shared state stores
│   ├── ipc/                 # Lock-free struct SHM readers/writers
│   └── utils/               # Distance estimator, logging
├── drpai/                   # C++ DRP-AI source (cross-compiled for V2N)
├── deploy/                  # Pre-built DRP-AI binary + model
├── scripts/                 # Shell scripts: start, sync, setup
├── launch/                  # ROS2 launch files (bringup, record)
├── urdf/                    # Robot URDF model
├── test/                    # Tests: unit/, hardware tests
└── docs/                    # Documentation
```

---

## Key Files

| File | Purpose |
|------|---------|
| `target_nav/config.py` | All parameters (defaults + validation ranges) |
| `target_nav/app/main.py` | Entry point — spawns 3 processes |
| `target_nav/app/nav_node.py` | ROS2 node + 20Hz control loop (Core 1) |
| `target_nav/app/camera_node.py` | Camera ROS2 node — DRP-AI subprocess (Core 2) |
| `target_nav/app/nav_controller.py` | Navigation state machine |
| `target_nav/app/target_tracker.py` | Kalman-filtered target tracker + LiDAR fusion |
| `target_nav/nav/navigator.py` | VFH, blind approach, spiral search, arrival |
| `target_nav/ipc/shm_struct.py` | Lock-free SPSC struct SHM |

---

## Deploy to V2N

```bash
# Sync code + rebuild (recommended for daily development):
./scripts/sync.sh

# Restart on V2N:
ssh root@192.168.50.1 "systemctl restart robot"
```

For first-time setup, see [V2N_SETUP_GUIDE.md](docs/V2N_SETUP_GUIDE.md). For development workflow, see [DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md).

---

## TF Tree

```
odom ──► base_link ──► laser
  │           └──► camera_link
  └── odometry_node (encoder ticks → odom frame)
```

Localization is odometry-only (no SLAM). The `odom` frame is the fixed reference frame.

---

## ROS2 Topics

| Topic | Type | Node | Direction | Purpose |
|-------|------|------|-----------|---------|
| `/cmd_vel` | Twist | nav_node | Publish | Motor velocity commands |
| `/scan` | LaserScan | rplidar_ros | Publish | LiDAR scan points |
| `/odom` | Odometry | odometry_node | Publish | Wheel odometry |
| `/arduino/cmd` | String | nav_node | Publish | Raw Arduino commands |
| `/arduino/odom_raw` | String | arduino_node | Publish | Raw encoder telemetry |
| `/reset_odom` | Empty | nav_node | Publish | Odometry reset trigger |
| `/nav_state` | String | nav_node | Publish | Nav state snapshot (JSON backup) |
| `/settings_changed` | String | nav_node, camera_node | Subscribe | Reload calibration |
| `/detections` | String | camera_node | Publish | Detection list (JSON backup) |
| `/detector_mode` | String | camera_node | Publish | Current detector mode |
| `/drpai_restart` | String | camera_node | Subscribe | Restart DRP-AI subprocess |

Note: Primary data flow uses struct SHM, not ROS2 topics. GUI has no ROS2 dependency.

---

## License

MIT
