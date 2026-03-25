# Developer Guide

> How to set up, develop, test, and deploy target_nav.

---

## Prerequisites

- **PC**: Ubuntu 22.04+, Python 3.10+, ROS2 Humble (optional for full tests)
- **V2N**: Renesas RZ/V2N board with BSP + ROS2 Humble (see [V2N_SETUP_GUIDE.md](V2N_SETUP_GUIDE.md))

---

## 1. Clone and Setup

```bash
git clone <repo-url> bowling_target_nav
cd bowling_target_nav

# Install Python dependencies (PC)
pip install numpy opencv-python pyserial pytest
```

No ROS2 required for unit tests — mocks are included.

---

## 2. Project Layout

```
bowling_target_nav/
├── target_nav/          # Python package (the main code)
│   ├── app/             # Entry points: main.py, nav_node.py, camera_node.py
│   ├── nav/             # Navigation algorithms: VFH, blind approach, search, arrival
│   ├── detectors/       # DRP-AI detection pipeline
│   ├── hardware/        # Arduino, Camera, LiDAR drivers (with mocks)
│   ├── gui/             # GTK3 interface: panels/, settings_tabs/
│   ├── state/           # Thread-safe shared state stores
│   ├── ipc/             # Lock-free struct SHM readers/writers
│   └── utils/           # Distance estimator, logging
├── drpai/               # C++ DRP-AI source (builds on V2N Docker)
├── deploy/              # Pre-built DRP-AI binary + model
├── scripts/             # Shell scripts: start, sync, setup
├── launch/              # ROS2 launch files
├── urdf/                # Robot URDF model
├── test/                # Tests: unit/, utils/, hardware tests
├── docs/                # Documentation
├── config.py            # NOT here — it's inside target_nav/
├── setup.py             # ROS2 ament_python package setup
└── package.xml          # ROS2 package manifest
```

### Key files to know

| File | What it does |
|------|-------------|
| `target_nav/config.py` | Single source of truth for ALL parameters |
| `target_nav/app/main.py` | Entry point — spawns 3 processes |
| `target_nav/nav/navigator.py` | Core navigation + algorithm constants |
| `target_nav/app/nav_controller.py` | State machine (IDLE→NAVIGATING→...) |
| `target_nav/app/target_tracker.py` | Kalman filter + LiDAR fusion |
| `target_nav/app/camera_worker.py` | DRP-AI detection pipeline |

---

## 3. Running Tests

### Unit tests (no hardware, no ROS2)

```bash
# Run all unit tests
pytest test/unit/ -v

# Run specific test
pytest test/unit/test_navigator.py -v

# Run with coverage
pytest test/unit/ --cov=target_nav --cov-report=term-missing
```

Unit tests use mocks for Arduino, LiDAR, camera, and ROS2. They work on any PC.

### Hardware tests (requires connected hardware)

```bash
# Check what hardware is available
./scripts/run_tests.sh --check

# Run specific hardware test
./scripts/run_tests.sh arduino    # Arduino motor test
./scripts/run_tests.sh lidar      # LiDAR scan test
./scripts/run_tests.sh camera     # Camera capture test

# Run all tests
./scripts/run_tests.sh all

# Interactive visualization
./scripts/run_tests.sh --visualize-lidar
./scripts/run_tests.sh --visualize-camera
```

Tests auto-skip when hardware is not available (via `conftest.py` markers).

---

## 4. Development Workflow

### Edit → Test → Deploy cycle

```bash
# 1. Edit code on PC
vim target_nav/nav/navigator.py

# 2. Run unit tests (instant feedback)
pytest test/unit/test_navigator.py -v

# 3. Deploy to V2N and rebuild
./scripts/sync.sh

# 4. Test on the robot
ssh root@192.168.50.1
systemctl restart robot    # Restart drivers
/root/gui.sh               # Launch GUI
```

### Quick file sync (skip full rebuild)

```bash
# Sync just one file (faster for small changes)
./scripts/sync.sh target_nav/nav/navigator.py
```

### Why rebuild is required

The V2N uses `colcon build` (not `--symlink-install`) because setuptools copies files to `install/`. Without rebuilding, the old code runs. `sync.sh` handles this automatically.

---

## 5. Common Development Tasks

### Change a navigation parameter default

Edit `target_nav/config.py` → `DEFAULT_NAV_PARAMS`:
```python
'linear_speed': 0.20,    # change this default
```
Then add the validation range in `PARAM_RANGES`:
```python
'linear_speed': (0.05, 0.60),  # allowed range
```
The GUI slider range is in `target_nav/gui/settings_tabs/nav_tab.py`.

### Add a new navigation parameter

1. Add default to `config.py` → `DEFAULT_NAV_PARAMS`
2. Add validation range to `config.py` → `PARAM_RANGES`
3. Add slider to `gui/settings_tabs/nav_tab.py`
4. Read it in `navigator.py` → `refresh_params()`

### Tune the Kalman filter

Edit `target_nav/config.py` → `DEFAULT_TRACKER_PARAMS`:
- `kf_q_*`: Process noise — higher = trusts measurements more, lower = smoother
- `kf_r_*`: Measurement noise — higher = trusts model more, lower = follows detections
- `size_gate_tolerance`: Reject detections with wrong bbox size (0.5 = ±50%)
- `lidar_angle_window`: Angular tolerance for LiDAR matching (degrees)

### Modify detection filtering

- **C++ side** (confidence, NMS): Edit `deploy/config.ini` or Settings → DRP-AI
- **Python temporal tracking**: Edit `config.py` → `DEFAULT_TRACKER_FILTER`
- **Shape validation**: Edit `camera_worker.py` → `_run_stream_mode()` shape filter section

---

## 6. Architecture Quick Reference

```
Process 0 (Core 0): GUI         — Pure GTK3, no ROS2, reads SHM
Process 1 (Core 1): Nav         — ROS2, 20Hz control loop, nice -5
Process 2 (Core 2): Camera      — DRP-AI subprocess management
Core 3:             DRP-AI C++  — YOLO inference (app_yolo_cam)
```

Data flows through lock-free struct SHM (no JSON, no Bridge process):
```
Camera → DetShmWriter → Nav (DetShmReader) + GUI (DetShmReader)
Nav → NavShmWriter → GUI (NavShmReader)
Nav → LaserShmWriter → GUI (LaserShmReader)
GUI → CmdRingBuffer → Nav (CmdRingBuffer)
```

See [ARCHITECTURE.md](ARCHITECTURE.md) for full details.

---

## 7. Debugging Tips

### View live ROS2 data (from PC over WiFi)

```bash
# Connect to RZV2N_Robot WiFi (password: robot1234)
export ROS_DOMAIN_ID=0
ros2 topic list
ros2 topic echo /nav_state          # Navigation state JSON
ros2 topic echo /detections          # Detection results
ros2 topic echo /odom --once         # Current odometry
ros2 topic hz /scan                  # LiDAR rate
```

### View logs on V2N

```bash
ssh root@192.168.50.1
journalctl -u robot -f               # Live driver logs
cat /var/log/robot_autostart.log      # Boot log
```

### Debug navigation

Set `LOG_INTERVAL_SECS = 0.5` in `navigator.py` for more frequent logging, then watch:
```bash
ros2 topic echo /nav_state | grep nav_state
```

### Debug detection

```bash
ros2 topic echo /detections           # See raw detections
ros2 topic echo /detector_mode        # Current DRP-AI mode
```
