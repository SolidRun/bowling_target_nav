# Hardware Setup Guide

> Complete hardware wiring, Arduino protocol, sensor configuration, and physical robot specifications.

---

## Table of Contents

1. [Physical Robot Specifications](#1-physical-robot-specifications)
2. [Hardware Components](#2-hardware-components)
3. [Wiring and Connections](#3-wiring-and-connections)
4. [Arduino Motor Controller Protocol](#4-arduino-motor-controller-protocol)
5. [LiDAR Configuration](#5-lidar-configuration)
6. [Camera Configuration](#6-camera-configuration)
7. [URDF Frame Definitions](#7-urdf-frame-definitions)
8. [Mecanum Wheel Kinematics](#8-mecanum-wheel-kinematics)
9. [Cartographer SLAM Tuning](#9-cartographer-slam-tuning)
10. [Troubleshooting Hardware](#10-troubleshooting-hardware)

---

## 1. Physical Robot Specifications

```
            ┌─────────────────────────┐
            │      USB Camera          │  ← /dev/video0 (front-facing, 640x480)
            │     ┌───────────┐        │
            │     │  RPLidar  │        │  ← /dev/ttyUSB0 (360° 2D laser)
            │     │    A1     │        │
            │     └───────────┘        │
            │                          │
  ┌────┐    │   RZ/V2N Board           │    ┌────┐
  │ FL │────│   + Motor Shield         │────│ FR │
  │    │    │   (PCA9685 I2C)          │    │    │
  └────┘    │                          │    └────┘
            │   Arduino                │
            │   /dev/ttyACM0           │
  ┌────┐    │                          │    ┌────┐
  │ RL │────│   Battery                │────│ RR │
  │    │    │                          │    │    │
  └────┘    └─────────────────────────┘    └────┘
```

### Dimensions

| Parameter | Value | Description |
|-----------|-------|-------------|
| Base size | 250 x 260 x 150 mm | Robot body (L x W x H) |
| Base mass | ~5.0 kg | Approximate total weight |
| Wheel diameter | 80 mm (radius 40 mm) | Mecanum wheels |
| Wheelbase | 190 mm | Front-rear axle distance |
| Track width | 210 mm | Left-right wheel distance |
| LiDAR height | 150 mm above base | Mounted on top |
| Camera offset | 120 mm forward, 100 mm above base | Front-facing |

### Wheel Layout (Top View)

```
        FRONT
   FL ─────── FR
   │           │
   │   ROBOT   │
   │           │
   RL ─────── RR
        REAR

Wheel indices (firmware order):
  FL = index 0
  RL = index 1
  RR = index 2
  FR = index 3
```

---

## 2. Hardware Components

| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Compute board | Renesas RZ/V2N | — | Main computer (Yocto Linux, ROS2 Humble) |
| Motor shield | QGPMaker (PCA9685 I2C) | I2C addr 0x60 | PWM generation for H-bridges |
| H-bridges | TB67H450 (x4) | — | Motor drivers (one per wheel) |
| Motors | DC with encoders (x4) | — | Mecanum wheel drive |
| Encoders | 4320 CPR (x4) | — | Wheel rotation sensing |
| Microcontroller | Arduino (Mega/Uno) | USB Serial | Motor control + encoder reading |
| LiDAR | RPLidar A1 | USB Serial | 360° 2D laser scanner |
| Camera | USB camera (generic) | USB V4L2 | Front-facing for YOLO detection |
| Display | HDMI monitor | HDMI | GUI output (Wayland/Weston) |
| Network | WiFi AP | WiFi | Access point at 192.168.50.1 |

---

## 3. Wiring and Connections

### USB Ports

| Device | Port | Baud Rate | Protocol |
|--------|------|-----------|----------|
| Arduino | `/dev/ttyACM0` | 115200 | Plain text commands |
| LiDAR | `/dev/ttyUSB0` | 115200 | rplidar protocol |
| Camera | `/dev/video0` | — | V4L2 video |

### Arduino Auto-Detection

The system can auto-detect the Arduino by USB VID/PID:

| Manufacturer | VID |
|-------------|-----|
| Arduino | 0x2341 |
| QinHeng (CH340) | 0x1A86 |
| Silicon Labs (CP210x) | 0x10C4 |
| FTDI | 0x0403 |

### Motor Shield I2C

```
V2N Board ──► Arduino ──► PCA9685 Motor Shield (I2C addr 0x60)
                              ├── TB67H450 H-bridge → FL motor (index 0)
                              ├── TB67H450 H-bridge → RL motor (index 1)
                              ├── TB67H450 H-bridge → RR motor (index 2)
                              └── TB67H450 H-bridge → FR motor (index 3)
```

### Encoder Connections

Each motor has a quadrature encoder (4320 counts per revolution). The Arduino reads all 4 encoders and reports values via serial.

---

## 4. Arduino Motor Controller Protocol

The Arduino firmware is at `/media/shtewe/extSSD/V2N_demoe/rzv2n-arduino-motor-controller/` and uses **PlatformIO** (NOT Arduino IDE).

### Command Format

**Plain text, newline-terminated, NO checksums.**

```
COMMAND[,arg1,arg2,...]\n
```

### Movement Commands

| Command | Format | Description | Requirements |
|---------|--------|-------------|--------------|
| FWD | `FWD,speed,ticks` | Move forward | speed > 0, ticks > 0 |
| BWD | `BWD,speed,ticks` | Move backward | speed > 0, ticks > 0 |
| LEFT | `LEFT,speed,ticks` | Strafe left | speed > 0, ticks > 0 |
| RIGHT | `RIGHT,speed,ticks` | Strafe right | speed > 0, ticks > 0 |
| TURN | `TURN,speed,ticks` | Rotate in place | speed > 0, +ticks=CCW, -ticks=CW |
| DIAGFL | `DIAGFL,speed,ticks` | Diagonal front-left | speed > 0, ticks > 0 |
| DIAGFR | `DIAGFR,speed,ticks` | Diagonal front-right | speed > 0, ticks > 0 |
| DIAGBL | `DIAGBL,speed,ticks` | Diagonal back-left | speed > 0, ticks > 0 |
| DIAGBR | `DIAGBR,speed,ticks` | Diagonal back-right | speed > 0, ticks > 0 |

### Continuous Velocity Command (Best for Navigation)

```
VEL,vx,vy,wz
```

| Parameter | Range | Description |
|-----------|-------|-------------|
| vx | -255 to 255 | Forward/backward PWM |
| vy | -255 to 255 | Left/right strafe PWM |
| wz | -255 to 255 | Rotation PWM (+ = CCW) |

**200ms Watchdog**: Motors automatically stop if no `VEL` command is received within 200ms. The ROS node must resend at >=5 Hz.

**Dead Zone**: Commands with `|vx|, |vy|, |wz| < 10` trigger automatic STOP.

### Simple Commands

| Command | Description |
|---------|-------------|
| `STOP` | Emergency stop all motors |
| `READ` | Read 4 encoder values (returns 4 lines: FL, RL, RR, FR) |
| `SYNC` | Synchronize communication |
| `CALIB` | Start motor calibration (~15 seconds) |

### Firmware Responses

| Response | Meaning |
|----------|---------|
| `READY` | Arduino booted and ready |
| `OK` | Command accepted |
| `DONE` | Operation completed |
| `BUSY` | Motors busy with previous command |
| `ERROR: msg` | Error with description |

### Telemetry (20Hz in VEL mode)

```
ODOM,vx_mm,vy_mm,wz_mrad      # Odometry velocities
ENC,FL:ticks,FR:ticks,RL:ticks,RR:ticks,t_us:microseconds
CALIB,progress                  # During calibration
STALL,motor_info                # Stall detection
```

### Critical Requirements

- **speed MUST be > 0** for all movement commands (FWD, BWD, etc.)
- **ticks MUST be > 0** for timed movements (ticks=0 causes firmware failure)
- **200ms watchdog** in VEL mode — must resend within 200ms or motors stop
- **No checksums** — plain ASCII text, newline-terminated
- **Baudrate**: 115200
- **Minimum PWM**: 20 (below this, motors stall due to friction)
- **Maximum PWM**: 255

### Connection Sequence

```
Python                          Arduino
  │                                │
  │ ─── Open serial 115200 ──────►│
  │                                │ (Arduino resets on DTR)
  │                    (wait 5s)   │
  │ ◄────────── "READY\n" ────────│
  │                                │
  │ ─── "VEL,100,0,0\n" ────────►│
  │ ◄────────── "OK\n" ──────────│
  │ ◄────── "ODOM,..." (20Hz) ───│
  │                                │
  │ ─── "STOP\n" ───────────────►│
  │ ◄────────── "DONE\n" ────────│
```

### Velocity Conversion (Python Side)

```python
# From m/s and rad/s to PWM values:
max_wheel_radps = 11.2
max_linear = max_wheel_radps * WHEEL_RADIUS  # ~0.448 m/s
max_angular = max_linear / ((WHEELBASE + TRACK_WIDTH) / 2)  # ~2.24 rad/s

vx_pwm = int(linear_x / max_linear * 255)   # Clamp to [-255, 255]
vy_pwm = int(linear_y / max_linear * 255)
wz_pwm = int(angular_z / max_angular * 255)

# Send: VEL,vx_pwm,vy_pwm,wz_pwm
```

---

## 5. LiDAR Configuration

### RPLidar A1 Specifications

| Parameter | Value |
|-----------|-------|
| Range | 0.15 – 12.0 m |
| Frequency | ~5.5 Hz (scan rate) |
| Angular resolution | ~1° (360 points/scan) |
| Interface | USB Serial, 115200 baud |
| Port | `/dev/ttyUSB0` |

### ROS2 Driver

The `rplidar_ros` package publishes `/scan` (sensor_msgs/LaserScan).

### Cartographer Filter Settings

```lua
-- In config/cartographer.lua:
TRAJECTORY_BUILDER_2D.min_range = 0.1   -- Ignore < 10cm
TRAJECTORY_BUILDER_2D.max_range = 8.0   -- Ignore > 8m
```

### Software Filter (in Navigator)

```python
# LiDAR points are filtered:
# - NaN/Inf values → skipped
# - x <= 0 → behind robot, skipped
# - x > slowdown_distance → too far, skipped
# - |y| > robot_half_width + 0.4m → outside corridor, skipped
```

---

## 6. Camera Configuration

### Default Settings

| Parameter | Value |
|-----------|-------|
| Device | `/dev/video0` (device_id=0) |
| Backend | V4L2 (`cv2.CAP_V4L2`) |
| Resolution | 640 x 480 |
| FPS | 30 |
| Buffer size | 1 (minimize latency) |
| Color format | BGR (OpenCV default) |

### Why V4L2 (Not GStreamer)

The V2N board's GStreamer can conflict with DRP-AI hardware acceleration. Using V4L2 directly avoids these issues.

### Camera Thread Behavior

1. Opens camera with retry (3 attempts, 1s between)
2. Captures at ~30fps
3. Submits frames to YOLO detector asynchronously
4. Draws cached detections on display frame
5. Auto-reconnects after 30 consecutive read failures

### Distance Estimation (Pinhole Camera Model)

```
distance = reference_distance * (reference_bbox_height / current_bbox_height)
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `reference_bbox_height` | 100 px | Bounding box height at reference distance |
| `reference_distance` | 1.0 m | Known distance for calibration |
| `horizontal_fov` | 60° | Camera field of view |

### Angle Estimation

```
focal_length = (frame_width / 2) / tan(fov / 2)
offset_x = center_x - frame_width / 2
angle = atan2(offset_x, focal_length)
```

---

## 7. URDF Frame Definitions

The robot's physical dimensions are defined in `urdf/v2n_robot.urdf`.

### Frame Hierarchy

```
base_footprint (ground plane)
  └── base_link (robot center, 0mm above ground)
      ├── laser (LiDAR sensor)
      │   Position: (0, 0, +150mm) — centered, 15cm above base
      │
      ├── camera_link (front camera)
      │   Position: (+120mm, 0, +100mm) — 12cm forward, 10cm above
      │
      ├── wheel_fl (front-left wheel)
      │   Position: (+95mm, +105mm, 0) — visual only
      │
      ├── wheel_fr (front-right wheel)
      │   Position: (+95mm, -105mm, 0) — visual only
      │
      ├── wheel_rl (rear-left wheel)
      │   Position: (-95mm, +105mm, 0) — visual only
      │
      └── wheel_rr (rear-right wheel)
          Position: (-95mm, -105mm, 0) — visual only
```

### Important Notes

- Wheel frames are **visual only** — no joints for simulation
- The `base_link → laser` transform tells Cartographer where the LiDAR is
- The `base_link → camera_link` transform is used for vision → base_link coordinate conversion
- `robot_state_publisher` broadcasts these as static TF transforms

---

## 8. Mecanum Wheel Kinematics

### Inverse Kinematics (Body Velocity → Wheel Speeds)

```
Given: vx (forward), vy (strafe), wz (rotation)

L = (wheelbase + track_width) / 2 = (0.190 + 0.210) / 2 = 0.200 m

FL = (vx - vy - L * wz) / wheel_radius
FR = (vx + vy + L * wz) / wheel_radius
RL = (vx + vy - L * wz) / wheel_radius
RR = (vx - vy + L * wz) / wheel_radius
```

### Motion Examples

| Motion | vx | vy | wz | FL | FR | RL | RR |
|--------|----|----|----|----|----|----|-----|
| Forward | + | 0 | 0 | + | + | + | + |
| Backward | - | 0 | 0 | - | - | - | - |
| Strafe Left | 0 | + | 0 | - | + | + | - |
| Strafe Right | 0 | - | 0 | + | - | - | + |
| Rotate CCW | 0 | 0 | + | - | + | - | + |
| Rotate CW | 0 | 0 | - | + | - | + | - |
| Diagonal FL | + | + | 0 | 0 | + | + | 0 |
| Diagonal FR | + | - | 0 | + | 0 | 0 | + |

### Forward Kinematics (Wheel Speeds → Body Velocity)

```
vx = (FL + FR + RL + RR) / 4 * wheel_radius
vy = (-FL + FR + RL - RR) / 4 * wheel_radius
wz = (-FL + FR - RL + RR) / (4 * L) * wheel_radius
```

### Speed Limits

| Parameter | Value | Description |
|-----------|-------|-------------|
| Max wheel speed | 11.2 rad/s | At PWM 255 |
| Max linear speed | 0.448 m/s | `11.2 * 0.04` |
| Max angular speed | 2.24 rad/s | `0.448 / 0.200` |
| Software linear limit | 0.30 m/s | Configurable |
| Software angular limit | 0.50 rad/s | Configurable |
| Minimum PWM | 20 | Below this, motors stall |

### Odometry Covariance

| Parameter | Value | Notes |
|-----------|-------|-------|
| Position X | 0.005 m | Forward direction |
| Position Y | 0.010 m | Lateral (more slip) |
| Heading | 0.01 rad | Gyro-less drift |
| Velocity X | 0.01 m/s | |
| Velocity Y | 0.02 m/s | Noisier strafe |
| Angular velocity | 0.02 rad/s | |

---

## 9. Cartographer SLAM Tuning

### Key Parameters (`config/cartographer.lua`)

```lua
-- Frames
map_frame = "map"
tracking_frame = "base_link"
odom_frame = "odom"
provide_odom_frame = true        -- Cartographer publishes map→odom

-- Sensor inputs
use_odometry = true              -- Wheel encoder integration
use_imu_data = false             -- No IMU on this robot
num_laser_scans = 1              -- Single 2D LiDAR

-- LiDAR range
min_range = 0.1                  -- Ignore below 10cm
max_range = 8.0                  -- Ignore beyond 8m

-- Map resolution
resolution = 0.05                -- 5cm per grid cell

-- Scan matching
use_online_correlative_scan_matching = true  -- Brute-force alignment

-- Motion filtering
max_distance_meters = 0.1        -- Minimum movement to add scan
max_angle_radians = 0.00873      -- ~0.5° minimum rotation

-- Submap size
num_range_data = 90              -- Scans per submap

-- Global SLAM (DISABLED for CPU efficiency)
optimize_every_n_nodes = 0       -- No loop closure

-- Publish rate
pose_publish_period_sec = 0.005  -- 200Hz pose updates
submap_publish_period_sec = 0.3  -- 3Hz map updates
```

### Why No Loop Closure?

Loop closure (`optimize_every_n_nodes > 0`) runs a global optimizer that matches current scans against all previous submaps. On the V2N's ARM processor, this causes CPU spikes and latency. Since the bowling environment is typically small and the robot returns to start, drift is manageable without it.

### Tuning Tips

| Symptom | Adjustment |
|---------|-----------|
| Map jittery | Increase `num_range_data` (larger submaps, more stable) |
| Map drifts | Enable loop closure: `optimize_every_n_nodes = 35` |
| Robot pose jumps | Increase `lookup_transform_timeout_sec` |
| High CPU | Reduce `max_range`, increase `num_range_data` |
| Map not building | Check `/scan` topic: `ros2 topic echo /scan --once` |

---

## 10. Troubleshooting Hardware

### Verify All Devices

```bash
# Check all devices exist
ls -la /dev/ttyACM0 /dev/ttyUSB0 /dev/video0

# Check Arduino
stty -F /dev/ttyACM0 115200

# Check LiDAR
stty -F /dev/ttyUSB0 115200

# Check Camera
v4l2-ctl --device=/dev/video0 --all
```

### Arduino Not Responding

```bash
# Check if port exists
ls /dev/ttyACM*

# Check process using the port
fuser /dev/ttyACM0

# Kill process and retry
fuser -k /dev/ttyACM0

# Test manually
screen /dev/ttyACM0 115200
# Type: STOP (should get DONE)
# Type: READ (should get 4 encoder values)
# Exit: Ctrl+A, then K
```

### LiDAR Not Spinning

```bash
# Check port
ls /dev/ttyUSB*

# Test scan
ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0
```

### Camera Not Working

```bash
# List cameras
ls /dev/video*

# Test capture
v4l2-ctl --device=/dev/video0 --stream-mmap --stream-count=1

# Check OpenCV
python3 -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened()); cap.release()"
```

### Display Issues (GUI Not Showing)

```bash
# Check Weston
pgrep -x weston

# Check Wayland socket
ls /run/wayland-*

# Set variables manually
export GDK_BACKEND=wayland
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/run
```

### Motor Stalling or Not Moving

1. Check minimum PWM: speed must be > 20 (motor friction threshold)
2. Check ticks: must be > 0 for timed movements
3. Check watchdog: VEL commands must arrive within 200ms
4. Check I2C: `i2cdetect -y 1` should show device at 0x60
5. Check battery voltage: low battery causes stalls

### Permission Issues

```bash
# Add user to dialout group (for serial ports)
usermod -a -G dialout root

# Add user to video group (for camera)
usermod -a -G video root
```
