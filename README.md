# Bowling Target Navigation - RZ/V2N

SLAM mapping + Camera with YOLO detection + Autonomous navigation for RZ/V2N robot.

**One script does everything**: Setup, autostart service, and GUI with navigation.

## Documentation

| Guide | Description |
|-------|-------------|
| [Quick Start Guide](docs/quick_start_guide.html) | Setup, commands, and examples - get running fast |
| [Technical Guide](docs/technical_guide.html) | How it works, architecture, calibration, limitations |

## Quick Start on V2N

### First Time Setup (One Command)

```bash
# From V2N - run the master setup script
cd ~/ros2_ws/src/bowling_target_nav/scripts
./v2n_master_setup.sh
```

This single command will:
1. Build the ROS2 package
2. Install autostart service (robot starts on boot)
3. Start the GUI with SLAM + Camera + Navigation

### Daily Usage

After first setup, the robot **starts automatically on boot** with:
- GUI showing camera feed + SLAM map
- "GO" button to navigate to detected bowling pins
- Obstacle avoidance using LiDAR

To manually start/stop:
```bash
./v2n_master_setup.sh --start   # Start GUI
./v2n_master_setup.sh --stop    # Stop everything
./v2n_master_setup.sh --status  # Check status
```

## GUI Controls

The main GUI shows:
- **Left panel**: SLAM map with robot position and LiDAR points
- **Right panel**: Camera feed with YOLO detection (bowling pins highlighted)
- **Bottom**: Control buttons

| Control | Action |
|---------|--------|
| **GO button** / G key | Navigate to detected bowling pins |
| **STOP button** / Space | Emergency stop |
| Q / ESC | Quit |

## Installation on New V2N

### From PC (Copy + Setup)

```bash
# 1. Connect to V2N WiFi network
# 2. Copy package and run setup
scp -r bowling_target_nav root@192.168.50.1:~/ros2_ws/src/
ssh root@192.168.50.1 "cd ~/ros2_ws/src/bowling_target_nav/scripts && chmod +x *.sh && ./v2n_master_setup.sh"
```

### What the Setup Does

1. Verifies ROS2 Humble is installed
2. Builds the package with colcon
3. Installs systemd service for auto-start
4. Starts the GUI

## PC Remote Control

From your PC, you can control the robot remotely:

```bash
# 1. Connect to V2N WiFi network
# 2. Run controller
cd tools/
./run_controller.sh
```

**PC Controller Features:**
- WASD / Arrow Keys: Move robot
- Q/E: Rotate left/right
- Space: Emergency stop
- Speed sliders
- Arduino commands (SYNC, encoder read/reset)

## Hardware Requirements

| Device | Port | Description |
|--------|------|-------------|
| Arduino | `/dev/ttyACM0` | Motor controller (Mecanum 4WD) |
| LiDAR | `/dev/ttyUSB0` | RPLidar A1 for SLAM |
| Camera | `/dev/video0` | USB camera for YOLO detection |
| Display | HDMI | Wayland/Weston for GUI |

## Service Commands

```bash
systemctl status robot      # Check status
systemctl start robot       # Start robot
systemctl stop robot        # Stop robot
systemctl restart robot     # Restart robot
journalctl -u robot -f      # View logs
```

## ROS2 Topics

| Topic | Description |
|-------|-------------|
| `/cmd_vel` | Velocity commands |
| `/scan` | LiDAR data |
| `/map` | SLAM map |
| `/odom` | Odometry |
| `/arduino/cmd` | Arduino commands |

## Troubleshooting

### Robot Not Responding

```bash
# Check hardware
ls -la /dev/ttyACM0 /dev/ttyUSB0 /dev/video0

# Release ports and restart
./v2n_master_setup.sh --stop
./v2n_master_setup.sh --start
```

### GUI Not Starting

```bash
# Check service logs
journalctl -u robot -n 50

# Manual start for debugging
./v2n_master_setup.sh --start
```

### Rebuild Package

```bash
./v2n_master_setup.sh --build
```

## Features

- **Auto-Start on Boot**: Robot ready in ~20 seconds
- **Unified GUI**: Map + Camera + Navigation in one window
- **Pluggable Detection**: Swap between YOLO ONNX and V2N DRP binary
- **GO Button**: One-click navigation to detected targets
- **Obstacle Avoidance**: LiDAR-based path planning
- **SLAM Mapping**: Real-time 2D map using Cartographer
- **Mecanum Drive**: Full omnidirectional control
- **PC Remote Control**: WiFi control from your PC
- **State Machine**: Clean state management (IDLE, DETECTING, NAVIGATING, etc.)
- **Configuration-Driven**: YAML config with environment variable overrides
- **Mock Components**: Test without hardware using mock implementations

## Detection Backends

Switch between detection backends via config or environment variable:

| Backend | Description | Config Value |
|---------|-------------|--------------|
| YOLO ONNX | Default, uses ONNX Runtime | `yolo_onnx` |
| DRP Binary | V2N hardware accelerated | `drp_binary` |
| Mock | Testing without camera | `mock` |

```bash
# Switch to DRP binary
export V2N_DETECTOR_TYPE=drp_binary

# Or edit config/robot_config.yaml:
# detection:
#   detector_type: "drp_binary"
```

## Testing

Comprehensive test suite with debug output and interactive GUIs.

### Run Tests

```bash
cd scripts/
./run_tests.sh              # Run all tests
./run_tests.sh arduino      # Arduino tests only
./run_tests.sh lidar        # LiDAR tests only
./run_tests.sh camera       # Camera + YOLO tests only
./run_tests.sh integration  # LiDAR + Camera tests
./run_tests.sh system       # Full system tests
./run_tests.sh --check      # Check hardware availability
```

### Interactive Test GUIs

```bash
./run_tests.sh --gui                # Full system control GUI
./run_tests.sh --visualize-lidar    # LiDAR visualization
./run_tests.sh --visualize-camera   # Camera detection demo
./run_tests.sh --visualize-fusion   # Sensor fusion view
```

### Test Files

| Test | Description |
|------|-------------|
| `test_arduino.py` | Arduino motor controller tests |
| `test_lidar.py` | LiDAR sensor tests |
| `test_camera.py` | Camera and YOLO detection tests |
| `test_lidar_camera.py` | Sensor fusion tests |
| `test_full_system.py` | Complete system tests with GUI |

## Package Structure

```
bowling_target_nav/
├── bowling_target_nav/
│   ├── core/                     # Core infrastructure
│   │   ├── config.py             # Configuration management
│   │   ├── state_machine.py      # Robot state machine
│   │   └── events.py             # Event bus
│   ├── detectors/                # Pluggable detection (Strategy pattern)
│   │   ├── base.py               # Abstract DetectorBase
│   │   ├── yolo_onnx_detector.py # YOLO ONNX implementation
│   │   ├── drp_binary_detector.py# V2N DRP binary support
│   │   ├── mock_detector.py      # Mock for testing
│   │   └── factory.py            # DetectorFactory
│   ├── hardware/                 # Hardware abstractions
│   │   ├── arduino.py            # Arduino (real + mock)
│   │   ├── camera.py             # Camera (real + mock)
│   │   └── lidar.py              # LiDAR (real + mock)
│   ├── nodes/
│   │   └── main_gui.py           # Main unified GUI
│   └── utils/
│       └── distance_estimator.py
├── test/
│   ├── unit/                     # Unit tests (no hardware)
│   │   ├── test_config.py
│   │   ├── test_state_machine.py
│   │   ├── test_detectors.py
│   │   └── test_hardware.py
│   └── hardware/                 # Hardware integration tests
│       ├── test_arduino.py
│       ├── test_lidar.py
│       └── test_camera.py
├── docs/
│   ├── quick_start_guide.html    # Quick setup & commands
│   └── technical_guide.html      # Detailed documentation
├── config/
│   ├── robot_config.yaml         # Main YAML configuration
│   └── cartographer.lua
├── scripts/
│   ├── v2n_master_setup.sh       # ONE SCRIPT FOR EVERYTHING
│   └── run_tests.sh              # Test runner
├── launch/
│   ├── bringup.launch.py
│   └── mapping.launch.py
├── tools/
│   └── pc_robot_controller.py
└── models/
    └── bowling_yolov5.onnx
```

## Network

V2N IP: `192.168.50.1` (WiFi hotspot)

```bash
ssh root@192.168.50.1
```
