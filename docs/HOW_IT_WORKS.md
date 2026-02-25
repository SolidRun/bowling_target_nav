# How the V2N Bowling Robot Works

A complete technical guide to the bowling pin navigation robot — from hardware to AI to autonomous navigation.

---

## Table of Contents

1. [The Physical Robot](#1-the-physical-robot)
2. [Boot Sequence](#2-boot-sequence)
3. [How the Robot Senses the World](#3-how-the-robot-senses-the-world)
   - [LiDAR — Seeing Obstacles in 360°](#3a-lidar--seeing-obstacles-in-360)
   - [Wheel Odometry — Knowing Where the Robot Is](#3b-wheel-odometry--knowing-where-the-robot-is)
   - [SLAM Mapping — Building a Map While Moving](#3c-slam-mapping--building-a-map-while-moving)
   - [Robot Pose — The TF Chain](#3d-robot-pose--the-tf-chain)
4. [How AI Pin Detection Works](#4-how-ai-pin-detection-works)
   - [Camera Thread Architecture](#4a-camera-thread-architecture)
   - [Detection Pipeline — Two Backends](#4b-detection-pipeline--two-backends)
   - [Distance and Angle Estimation](#4c-distance--angle-estimation)
5. [How Navigation Decisions Are Made](#5-how-navigation-decisions-are-made)
   - [The Control Loop](#5a-the-control-loop-20hz)
   - [State Machine](#5b-state-machine)
6. [How the Robot Actually Moves](#6-how-the-robot-actually-moves)
   - [Navigation Algorithm](#6a-navigation-algorithm)
   - [Arrival Detection](#6b-arrival-detection--how-the-robot-knows-it-reached-the-pin)
   - [Obstacle Avoidance](#6c-obstacle-avoidance)
   - [Blind Approach](#6d-blind-approach)
   - [From Twist to Wheels](#6e-from-twist-to-wheels)
   - [Mecanum Wheel Kinematics](#6f-mecanum-wheel-kinematics)
7. [How All Parts Connect](#7-how-all-parts-connect)
   - [The Three Threads](#7a-the-three-threads)
   - [Shared State — Thread-Safe Glue](#7b-shared-state--thread-safe-glue)
   - [ROS2 Topic Map](#7c-ros2-topic-map)
   - [Complete Data Flow — One Navigation Cycle](#7d-complete-data-flow--one-navigation-cycle)
8. [The GUI](#8-the-gui)
9. [Project File Structure](#9-project-file-structure)
10. [Design Patterns Used](#10-design-patterns-used)
11. [Safety Mechanisms](#11-safety-mechanisms)

---

## 1. The Physical Robot

```
            ┌─────────────────────────┐
            │      USB Camera          │  ← /dev/video0 (640x480, front-facing)
            │     ┌───────────┐        │
            │     │  RPLidar  │        │  ← /dev/ttyUSB0 (360° laser scanner)
            │     │    A1     │        │
            │     └───────────┘        │
            │                          │
            │    RZ/V2N SoC Board      │  ← ARM64 CPU + DRP-AI accelerator
            │    (runs Linux + ROS2)   │
            │                          │
      ┌─────┤                          ├─────┐
      │ FL  │    Arduino Mega          │ FR  │  ← /dev/ttyACM0
      │  ◉  │    + PCA9685 Shield     │  ◉  │     (I2C motor control)
      └─────┤                          ├─────┘
      ┌─────┤     250mm × 260mm       ├─────┐
      │ RL  │     (robot body)         │ RR  │
      │  ◉  │                          │  ◉  │
      └─────┴──────────────────────────┴─────┘
               4 Mecanum Wheels (40mm radius)
```

### Hardware Components

| Component | Device | Connection | Purpose |
|-----------|--------|------------|---------|
| **RZ/V2N Board** | Renesas RZ/V2N SoC | — | Main computer: ARM64 CPU + DRP-AI neural accelerator |
| **RPLidar A1** | Slamtec laser scanner | `/dev/ttyUSB0` | 360° distance scanning, 8KHz, 6m range (up to 12m for reflective targets) |
| **USB Camera** | Generic V4L2 camera | `/dev/video0` | 640×480 RGB, front-facing for pin detection |
| **Arduino Mega** | ATmega2560 | `/dev/ttyACM0` at 115200 baud | Motor control via PCA9685 I2C PWM board |
| **Motor Shield** | QGPMaker PCA9685 | I2C address 0x60 | Drives 4 TB67H450 H-bridges |
| **Mecanum Wheels** | 4× omnidirectional | — | Holonomic movement (forward, sideways, diagonal, spin) |
| **Wheel Encoders** | 4× quadrature | — | 4320 ticks per revolution for odometry |
| **Display** | MIPI-DSI screen | Weston/Wayland | Touchscreen GUI on the robot |

### What Makes Mecanum Wheels Special

Unlike normal wheels that can only go forward/backward, **mecanum wheels** have angled rollers at 45° around their circumference. By spinning each wheel at different speeds and directions, the robot can move in **any direction** without turning:

```
Forward:  All wheels forward       Strafe Left:  FL↓ FR↑ RL↑ RR↓
   ↑↑                                ← ←
   ↑↑                                → →

Rotate:   FL↓ FR↑ RL↓ RR↑         Diagonal:     FL↗ FR→ RL→ RR↗
   ↓↑                                 ↗ →
   ↓↑                                 → ↗
```

This is crucial for navigation — the robot can **strafe sideways** around obstacles while still facing the target.

---

## 2. Boot Sequence

When the robot powers on, a systemd service handles the entire startup:

```
Power On
   │
   ▼
systemd starts robot.service
   │
   ▼
robot_autostart.sh runs:
   │
   ├── 1. Wait up to 10s for hardware
   │       (/dev/ttyACM0 = Arduino, /dev/ttyUSB0 = LiDAR)
   │
   ├── 2. Source ROS2 environment
   │       (source /opt/ros/humble/setup.bash)
   │       (source ~/ros2_ws/install/setup.bash)
   │
   ├── 3. Kill any old processes
   │       (ros2, arduino_driver, rplidar, cartographer, main_gui)
   │
   ├── 4. Wait for Weston/Wayland display
   │       (the touchscreen compositor)
   │
   ├── 5. Start bringup.launch.py ──────────────────────────┐
   │       ├── robot_state_publisher  (URDF frame tree)     │
   │       ├── rplidar_node           (/scan topic)         │ Hardware
   │       ├── arduino_driver_node    (motor control)       │ Drivers
   │       └── odometry_node          (/odom + TF)          │
   │                                                        │
   ├── 6. Start mapping.launch.py (3s delay) ───────────────┤
   │       ├── cartographer_node           (SLAM)           │ Mapping
   │       └── cartographer_occupancy_grid (grid map)       │
   │                                                        │
   └── 7. Start main_gui (2s delay) ───────────────────────┤
           ├── GTK3 fullscreen window  (Wayland display)    │ Application
           ├── ROS2 node thread        (navigation)         │
           └── Camera thread           (AI detection)       │
```

### The Launch Files

The system uses ROS2 launch files to manage process groups:

- **`bringup.launch.py`** — Core hardware: LiDAR, Arduino, Odometry, TF (robot frame tree)
- **`mapping.launch.py`** — Cartographer SLAM for map building
- **`main_gui`** — The application entry point (not a launch file, runs directly)

### Why the Delays?

- **3s between bringup and mapping**: Cartographer needs `/scan` and `/odom` topics to be publishing before it starts. Without the delay, it would crash.
- **2s between mapping and GUI**: The GUI's ROS2 node needs the TF tree (`map→odom→base_link`) to be established by Cartographer.

---

## 3. How the Robot Senses the World

The robot combines three sensor sources to understand its environment:

### 3a. LiDAR — Seeing Obstacles in 360°

The RPLidar A1 is a spinning laser that measures distance in every direction, 5.5-10 times per second.

```
                     0° (front)
                      │
              ┌───────┼───────┐
              │   ·   │   ·   │
      270° ───┤ · · ─ ● ─ · · ├─── 90°
     (right)  │   ·   │   ·   │   (left)
              └───────┼───────┘
                      │
                   180° (back)

Each '·' is a laser range measurement.
360 points per scan, each with (angle, distance).
```

**In code** (`ros_node.py` → `scan_cb`):

1. RPLidar publishes a `LaserScan` message on `/scan` with ~360 range values
2. The callback converts each (angle, range) into (x, y) cartesian coordinates in the robot's frame:
   ```
   x = range × cos(angle)    ← how far ahead/behind
   y = range × sin(angle)    ← how far left/right
   ```
3. These points are stored in `state.sensors.set_laser(points)` for:
   - The **navigator** to check for obstacles
   - The **GUI** to draw laser dots on the map

### 3b. Wheel Odometry — Knowing Where the Robot Is

Each wheel has an encoder that counts ticks as it rotates (4320 ticks = one full wheel revolution). By reading how many ticks each wheel has turned, the robot calculates how it moved.

```
Arduino Firmware            arduino_driver_node          odometry_node
      │                            │                           │
      │ Serial: "ENC,FL:1234,      │ /arduino/odom_raw         │
      │  FR:1230,RL:1240,RR:1235"  │ (JSON string)            │
      └────────────────────────────►│─────────────────────────►│
                                                               │
                                        Mecanum Forward Kinematics:
                                        ┌────────────────────────────┐
                                        │ R = wheel radius (0.04m)   │
                                        │ L = wheelbase/2 + track/2  │
                                        │                            │
                                        │ vx = R/4×(wFL+wFR+wRL+wRR)│ ← forward speed
                                        │ vy = R/4×(-wFL+wFR+wRL-wRR)│ ← sideways speed
                                        │ wz = R/(4L)×(-wFL+wFR-wRL+wRR)│ ← rotation
                                        └────────────────────────────┘
                                                               │
                                        Integrate velocities over time:
                                        ┌────────────────────────────┐
                                        │ x += vx×cos(θ)×dt         │
                                        │      - vy×sin(θ)×dt       │
                                        │ y += vx×sin(θ)×dt         │
                                        │      + vy×cos(θ)×dt       │
                                        │ θ += wz×dt                │
                                        └────────────────────────────┘
                                                               │
                                        Publish:               │
                                        ├── /odom (Odometry msg)
                                        └── TF: odom → base_link
```

**Why mecanum kinematics is different from normal robots:**

A normal 2-wheel robot can only compute `vx` (forward) and `wz` (rotation). A mecanum robot also computes `vy` (sideways speed) because the angled rollers allow lateral movement. The formula relates all 4 wheel speeds to the 3 body velocities.

**Odometry drift:** Over time, small errors in wheel slip accumulate. After driving 10 meters, the robot's position estimate might be off by 10-20cm. This is why we need SLAM.

### 3c. SLAM Mapping — Building a Map While Moving

**SLAM** (Simultaneous Localization And Mapping) solves a chicken-and-egg problem: to build a map you need to know where you are, but to know where you are you need a map. Cartographer solves both at once.

```
/scan (LiDAR) ────────┐
                       ▼
              ┌────────────────┐
              │  Cartographer   │
              │     (SLAM)      │
              │                 │
/odom ────────►  How it works:  │
              │                 │
              │  1. Get new laser scan                               │
              │  2. Try to match it against the existing map         │
              │     (scan matching = "where do these wall shapes     │
              │      best align with walls already on the map?")     │
              │  3. Use odometry as initial guess for robot motion   │
              │  4. Refine the pose using scan match quality         │
              │  5. Add matched scan to the map                      │
              │  6. Correct odometry drift using scan alignment      │
              │                                                      │
              └──────┬─────────┘
                     │
                     ├── /map (OccupancyGrid)
                     │   ┌──────────────────────────────┐
                     │   │ Each 5cm×5cm cell is:        │
                     │   │  -1 = unknown (never seen)   │
                     │   │   0 = free space (no wall)   │
                     │   │ 100 = occupied (wall/object) │
                     │   └──────────────────────────────┘
                     │
                     └── TF: map → odom
                         (the correction for odometry drift)
```

**Our specific Cartographer configuration** (`config/cartographer.lua`):

| Setting | Value | Why |
|---------|-------|-----|
| `use_odometry = true` | Uses wheel encoders | Helps scan matching converge faster |
| `use_imu_data = false` | No IMU on this robot | We only have wheels and LiDAR |
| `max_range = 8.0m` | Ignore readings beyond 8m | RPLidar A1 is noisy at long range |
| `resolution = 0.05m` | 5cm grid cells | Good balance of detail vs memory |
| `optimize_every_n_nodes = 0` | No loop closure | Saves CPU on embedded ARM board |
| `use_online_correlative_scan_matching = true` | Brute-force initial match | More robust than pure odometry guess |

### 3d. Robot Pose — The TF Chain

ROS2 uses **TF (Transform Frames)** to track how coordinate frames relate to each other. The robot has this chain:

```
map ──(Cartographer)──► odom ──(odometry_node)──► base_link
 │                                                     │
 │  "Where is the robot                    ┌───────────┼────────────┐
 │   in the world?"                        ▼           ▼            ▼
 │                                       laser    camera_link    wheels
 │
 └── The robot_state_publisher reads the URDF file
     and publishes all the fixed transforms
     (laser position, camera position, wheel positions)
```

**How the GUI gets the robot's world position:**

```python
# In ros_node.py scan_cb:
t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
x = t.transform.translation.x      # meters in map frame
y = t.transform.translation.y
theta = atan2(2*(qw*qz + qx*qy),   # heading from quaternion
              1 - 2*(qy² + qz²))
```

This gives the robot's drift-corrected position by combining:
- `map→odom` (Cartographer's drift correction)
- `odom→base_link` (wheel odometry)

---

## 4. How AI Pin Detection Works

### 4a. Camera Thread Architecture

The camera and detection run in a **separate thread** from both the GUI and ROS2 to avoid blocking either one.

```
camera_worker.py (Camera Thread)
┌───────────────────────────────────────────────────────────────┐
│                                                               │
│  Main loop (runs continuously):                               │
│                                                               │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │  1. Grab frame from camera (fast, ~30fps)               │  │
│  │                                                         │  │
│  │  2. Check if previous detection finished:               │  │
│  │     └── YES: save results as cached_detections          │  │
│  │                                                         │  │
│  │  3. Should we submit new detection?                     │  │
│  │     (rate-limited: every 2.0s for DRP-AI)               │  │
│  │     └── YES: executor.submit(detect, frame.copy())      │  │
│  │              ↑ runs in background thread pool            │  │
│  │                                                         │  │
│  │  4. Expire old detections (>1.5s = stale)               │  │
│  │                                                         │  │
│  │  5. Draw bounding boxes on display frame                │  │
│  │                                                         │  │
│  │  6. Store in shared state:                              │  │
│  │     state.detection.set_camera(frame, detections, info) │  │
│  └─────────────────────────────────────────────────────────┘  │
│                                                               │
│  Why async? Detection takes 100-500ms. If we waited for it,  │
│  the camera feed would freeze. Instead, we show the latest    │
│  frame immediately and overlay cached detection boxes.        │
│                                                               │
└───────────────────────────────────────────────────────────────┘
```

### 4b. Detection Pipeline — Two Backends

The system tries two detection backends in order:

#### Primary: DRP-AI (Hardware Accelerated)

Available **only on the V2N board**, the DRP-AI is a dedicated neural network accelerator. It runs YOLO inference **10-20x faster** than the CPU.

```
Python (camera_worker.py)           C++ Binary (/home/root/deploy/yolo_detection)
        │                                          │
        │  stdin: [4 bytes width]                   │
        │         [4 bytes height]                  │
        │         [width×height×3 BGR pixels]       │
        │─────────────────────────────────────────►│
        │                                          │  Sends frame to
        │                                          │  DRP-AI hardware
        │                                          │  via Linux DRM/KMS
        │                                          │
        │  stdout: JSON line                        │
        │  {"detections":[                          │
        │    {"class_id":0,                         │
        │     "confidence":0.87,                    │
        │     "x1":120,"y1":80,                     │
        │     "x2":200,"y2":350}                    │
        │  ],"inference_ms":45.2}                   │
        │◄─────────────────────────────────────────│
```

The C++ binary is a persistent subprocess. It loads the YOLO model into DRP-AI memory once at startup, then processes frames through a stdin/stdout pipe. This avoids the overhead of starting a new process per frame.

#### Fallback: ONNX Runtime (CPU)

If DRP-AI is not available (e.g., developing on a laptop), the system falls back to ONNX Runtime running the same YOLO model on CPU. This is ~10x slower but works everywhere.

```
frame (640×480 BGR)
    │
    ▼
Preprocess:
    resize to 640×640
    BGR → RGB
    normalize to 0.0-1.0
    transpose to NCHW format [1, 3, 640, 640]
    │
    ▼
ONNX Runtime Inference (CPU)
    │
    ▼
Postprocess:
    Parse output tensor (YOLOv8 format: [1, 8, 2100])
    Filter by confidence threshold (> 0.35)
    Filter by class name ("bowling-pins")
    Apply NMS (Non-Maximum Suppression)
    Scale boxes back to original frame size
    │
    ▼
List[Detection(class_name, confidence, bbox)]
```

**What is NMS (Non-Maximum Suppression)?**

YOLO often produces multiple overlapping detections for the same object. NMS keeps only the highest-confidence box and removes boxes that overlap it by more than 45% (IoU threshold). This ensures each pin gets exactly one detection.

### 4c. Distance & Angle Estimation

Once we have a bounding box, we estimate how far away the pin is and at what angle.

```
                        Camera Frame (640×480)
                       ┌──────────────────────────────┐
                       │                              │
                       │         ┌────────┐           │
                       │         │        │           │
                       │         │  Pin   │ height    │
                       │         │  bbox  │ = h px    │
                       │         │        │           │
                       │         └────────┘           │
                       │          ▲                   │
                       │          │                   │
                       │    cx pixels from            │
                       │    frame center              │
                       │                              │
                       └──────────────────────────────┘
                       ◄────── FOV = 60° ──────────────►
```

**Distance formula (pinhole camera model):**

```
distance = reference_distance × (reference_box_height / current_box_height)
```

- Calibration default: 100px box height at 1.0m distance
- If the pin appears 200px tall → it's at 0.5m (closer = bigger)
- If the pin appears 50px tall → it's at 2.0m (farther = smaller)

The Settings GUI has a calibration tool: place a pin at a known distance, press CALIBRATE, and it updates the reference values.

**Angle formula:**

```
focal_length = (frame_width / 2) / tan(FOV / 2)
             = 320 / tan(30°)
             = 554 pixels

offset_x = detection_center_x - 320    (pixels from frame center)

angle = atan2(offset_x, focal_length)   (radians)
```

- Pin in the center → angle ≈ 0°
- Pin on the right edge → angle ≈ +30°
- Pin on the left edge → angle ≈ -30°

**LiDAR+Vision Fusion:**

The navigator doesn't trust vision distance alone (it depends on calibration). When available, it checks the LiDAR reading at the same angle:

```python
# In navigator.py navigate_to_target():
lidar_distance = state.sensors.get_lidar_distance_at_angle(angle)

if lidar_distance < infinity:
    distance = lidar_distance     # LiDAR is more accurate
    source = "lidar"
else:
    distance = vision_distance    # Fall back to vision estimate
    source = "vision"
```

---

## 5. How Navigation Decisions Are Made

### 5a. The Control Loop (20Hz)

The control loop runs every 50ms (20 times per second) in the ROS thread. It reads the latest sensor and detection data and decides what to do.

```
control_loop() — runs every 50ms in ros_node.py
┌──────────────────────────────────────────────────────────────────┐
│                                                                  │
│  Step 1: Check user commands                                     │
│  ┌─────────────────────────────────────────┐                     │
│  │ STOP pressed? → Stop motors, go IDLE    │                     │
│  │ GO pressed?   → Activate navigation     │                     │
│  │ Neither?      → Continue current task   │                     │
│  └─────────────────────────────────────────┘                     │
│                                                                  │
│  Step 2: Get latest detections                                   │
│  ┌─────────────────────────────────────────┐                     │
│  │ Read from state.detection.get_camera()  │                     │
│  │ Discard if detection is too old         │                     │
│  │ (older than tunable expiry, default 1.5s)│                    │
│  └─────────────────────────────────────────┘                     │
│                                                                  │
│  Step 3: Find best target                                        │
│  ┌─────────────────────────────────────────┐                     │
│  │ find_best_target(detections)            │                     │
│  │ → picks the closest pin by distance     │                     │
│  └─────────────────────────────────────────┘                     │
│                                                                  │
│  Step 4: Navigate based on target visibility                     │
│                                                                  │
│  ┌──── Target visible? ────┐                                     │
│  │                          │                                    │
│  YES                        NO                                   │
│  │                          │                                    │
│  ├─ Record "target seen"    ├─ How long since last seen?         │
│  │  timestamp               │                                    │
│  │                          ├─ < lost_timeout (3s):              │
│  ├─ If was SEARCHING:       │   Drift forward at min speed       │
│  │  log "Target found!"     │   (target probably briefly hidden) │
│  │                          │                                    │
│  ├─ If ARRIVED:             ├─ > lost_timeout AND close (<0.8m): │
│  │  Stop. Terminal state.   │   Enter BLIND_APPROACH             │
│  │  User presses GO again.  │   (dead-reckon to last position)   │
│  │                          │                                    │
│  ├─ Otherwise:              ├─ > lost_timeout AND far:           │
│  │  navigate_to_target()    │   Start SEARCHING (rotate)         │
│  │                          │                                    │
│  │                          └─ Search > 30s: Give up → IDLE      │
│  └──────────────────────────────────────────────────────────────│
└──────────────────────────────────────────────────────────────────┘
```

### 5b. State Machine

The navigation system has 5 states:

```
                    ┌─────── STOP pressed ──────────────────────┐
                    │                                           │
                    ▼         GO pressed                        │
              ┌──► IDLE ◄────────────── Search timeout (30s)   │
              │     │                                           │
              │     │ No target visible                         │
              │     ▼                                           │
              │  SEARCHING ─────── Target found! ──────┐       │
              │     ▲                                   │       │
              │     │                                   ▼       │
              │  Target lost >3s              NAVIGATING ◄──────┤
              │  AND far (>0.8m)                  │             │
              │     │                             │             │
              │     │                    Target lost >3s        │
              │     │                    AND close (<0.8m)      │
              │     │                             │             │
              │     │                             ▼             │
              │     └──────────────── BLIND_APPROACH            │
              │                             │                   │
              │                    Arrived / LiDAR stop         │
              │                             │                   │
              │                             ▼                   │
              └─────────────────────── ARRIVED ─────────────────┘
                                    (terminal state)
                                   User presses GO
                                   to restart
```

**State descriptions:**

| State | Color | What's Happening |
|-------|-------|------------------|
| **IDLE** | Gray | Waiting for user to press GO |
| **SEARCHING** | Yellow | Rotating in place looking for pins |
| **NAVIGATING** | Green | Driving toward a visible pin |
| **BLIND_APPROACH** | Orange | Dead-reckoning to last known pin position (camera lost it) |
| **ARRIVED** | Blue | Reached the target. Stopped. Terminal — press GO to restart. |

---

## 6. How the Robot Actually Moves

### 6a. Navigation Algorithm

When a pin is detected, the navigator computes a **holonomic velocity command** to drive toward it:

```python
# In navigator.py direct_navigate():

# 1. Target is at (target_x, target_y) in robot frame
#    target_x = forward distance
#    target_y = lateral distance (positive = left)
angle = atan2(target_y, target_x)

# 2. Compute drive speed (slow down when close)
speed_scale = min(1.0, distance / 0.40)    # ramp down in last 40cm
speed = linear_speed × max(0.4, speed_scale)

# 3. Holonomic velocity (this is the mecanum advantage!)
cmd.linear.x = speed × cos(angle)    # forward component
cmd.linear.y = speed × sin(angle)    # strafe component
cmd.angular.z = angle × 0.5          # gentle correction toward target

# 4. Enforce minimum speed (motors have a dead zone below ~0.10 m/s)
if total_speed > 0 and total_speed < min_linear_speed:
    scale_up = min_linear_speed / total_speed
    cmd.linear.x *= scale_up
    cmd.linear.y *= scale_up
```

**Example scenarios:**

| Pin Position | linear.x | linear.y | Result |
|-------------|----------|----------|--------|
| Directly ahead, 1m | 0.15 | 0.00 | Drive straight forward |
| 30° to the right, 1m | 0.13 | -0.075 | Drive forward-right diagonally |
| 45° to the left, 0.5m | 0.07 | 0.07 | Strafe diagonally forward-left |
| Directly left, 0.3m | 0.00 | 0.10 | Pure sideways strafe |

### 6b. Arrival Detection — How the Robot Knows It Reached the Pin

Determining "I have arrived" is harder than it sounds. The robot uses **multi-signal arrival detection** with temporal confirmation to avoid both false arrivals and missed arrivals.

**The problem with simple distance checks:**

A single distance measurement is noisy. Vision distance (based on bounding box height) can fluctuate by 5cm frame-to-frame. A single bad reading of 0.14m could trigger a false arrival, while a reading of 0.16m would miss a real arrival. Additionally, when the pin is very close, the bounding box gets clipped at the frame edge, making the height measurement unreliable.

**Three independent arrival signals:**

```
Signal 1: Fused Distance            Signal 2: LiDAR Frontal         Signal 3: LiDAR at Target Angle
(vision or LiDAR)                   (any obstacle in front)         (obstacle in pin's direction)

  Pin ← 0.12m → Robot                    ▓ 0.10m                       ↗ beam hits pin
  distance < 0.15m?                  ████████████                      at < 0.15m?
                                    ████ Robot ████
                                    ████████████
```

**How it works:**

```
navigate_to_target() called at 20Hz
    │
    ├── Compute fused distance (LiDAR preferred, vision fallback)
    │
    ├── _check_arrival() — evaluates all three signals
    │       │
    │       ├── Emergency override?
    │       │   LiDAR front < 0.12m AND (distance close OR bbox clipped)
    │       │   └── YES → Immediate ARRIVED (no waiting)
    │       │
    │       ├── Any signal says "close"?
    │       │   └── YES → Start confirmation timer
    │       │           Timer running ≥ 0.3s? → ARRIVED (confirmed!)
    │       │           Timer running < 0.3s? → Slow creep toward target
    │       │
    │       └── All signals say "far"?
    │           └── Distance > threshold × 1.5? → Reset timer (hysteresis)
    │
    ├── Bbox clipped + vision-only + close?
    │   └── Vision distance unreliable → Enter blind approach early
    │
    └── Not arrived → direct_navigate() (full speed navigation)
```

**Key design choices:**

| Mechanism | Value | Why |
|-----------|-------|-----|
| **Confirmation time** | 0.3s (6 ticks at 20Hz) | Filters single-frame noise without perceptible delay |
| **Hysteresis** | 1.5× threshold | Timer resets only when distance > 0.225m, preventing oscillation near 0.15m |
| **Creep during confirmation** | 50% of min speed | Robot still moves toward target during 0.3s, covering ~1.5cm — well within margin |
| **Bbox clipping margin** | 5px from frame edge | Pin touching frame border = height is underestimated = distance overestimated |
| **Emergency LiDAR stop** | 0.12m front + any close signal | Catches "pin right in front" even when vision is confused |

### 6c. Obstacle Avoidance

Before driving, the navigator checks the LiDAR for obstacles in the path:

```
               Front obstacle zone
          ┌─────────────────────────────┐
          │                             │
          │   slowdown zone (0.5m)      │
          │   ┌─────────────────────┐   │
          │   │ stop zone (0.25m)   │   │
          │   │                     │   │
          │   │   ┌─────────────┐   │   │
          │   │   │   Robot     │   │   │
          │   │   │  (0.15m     │   │   │
          │   │   │   half-     │   │   │
          │   │   │   width)    │   │   │
          │   │   └─────────────┘   │   │
          │   │                     │   │
     Left │   └─────────────────────┘   │ Right
     free?│                             │ free?
          └─────────────────────────────┘
```

**Decision logic:**

```
if obstacle < 0.25m (stop zone):
    if obstacle < 0.15m (emergency):
        if stuck > 3 seconds:
            → back up slowly
        else:
            → hold position (zero velocity)
    else if left is free:
        → strafe left + creep forward
    else if right is free:
        → strafe right + creep forward
    else:
        → both sides blocked, back up

else if obstacle < 0.5m (slowdown zone):
    → reduce speed proportionally
    → closer = slower

else:
    → full speed ahead
```

### 6d. Blind Approach

When the robot is close to a pin (<0.8m) but the camera loses it (pin exits the field of view, or detection fails), the robot enters **blind approach** mode:

```
Step 1: Save target position in map frame
        target_map = robot_pose + target_offset (rotated to map frame)

Step 2: Dead-reckon toward saved position
        ┌──────────────────────────────────────┐
        │  Every 50ms:                         │
        │  1. Read current robot pose from TF  │
        │  2. Calculate: remaining = distance   │
        │     from current pose to target_map   │
        │  3. Calculate: heading error           │
        │  4. Drive at 0.10 m/s toward target   │
        │  5. Check exit conditions              │
        └──────────────────────────────────────┘

Exit conditions:
  ✅ remaining < 0.10m        → ARRIVED (reached the pin!)
  ✅ LiDAR front < 0.12m      → ARRIVED (bumped into something)
  ❌ heading error > 45°       → ABORT → SEARCHING (lost track)
  ❌ timeout > 8s              → ABORT → SEARCHING
  ✅ target re-detected        → switch back to NAVIGATING
```

**Why blind approach exists:**

When the robot gets very close, the pin often exits the camera's field of view (the camera can't see straight down). Without blind approach, the robot would stop and start searching — spinning around looking for a pin that's right in front of it. Blind approach uses the last known position and wheel odometry to finish the approach.

### 6e. From Twist to Wheels

The complete chain from navigation decision to wheel motion:

```
Navigator                  ROS2              Arduino Driver         Arduino
    │                        │                     │                  │
    │  _publish_cmd(Twist)   │                     │                  │
    │  ┌─────────────────┐   │                     │                  │
    │  │ NaN/Inf guard   │   │                     │                  │
    │  │ (replace with 0)│   │                     │                  │
    │  │                 │   │                     │                  │
    │  │ Speed cap       │   │                     │                  │
    │  │ (vector mag     │   │                     │                  │
    │  │  ≤ linear_speed)│   │                     │                  │
    │  └─────────────────┘   │                     │                  │
    │          │              │                     │                  │
    │   publish on /cmd_vel   │                     │                  │
    │────────────────────────►│                     │                  │
    │                         │  cmd_vel callback   │                  │
    │                         │────────────────────►│                  │
    │                         │                     │                  │
    │                         │  Convert to PWM:    │                  │
    │                         │  vx_pwm = vx × 585  │                  │
    │                         │  vy_pwm = vy × 585  │                  │
    │                         │  wz_pwm = wz × 585  │                  │
    │                         │                     │                  │
    │                         │  Serial: "VEL,vx,vy,wz\n"             │
    │                         │                     │─────────────────►│
    │                         │                     │                  │
    │                         │                     │  Mecanum IK:     │
    │                         │                     │  FL = vx-vy-wz   │
    │                         │                     │  FR = vx+vy+wz   │
    │                         │                     │  RL = vx+vy-wz   │
    │                         │                     │  RR = vx-vy+wz   │
    │                         │                     │                  │
    │                         │                     │  PCA9685.setPWM() │
    │                         │                     │  for each motor   │
```

**The 200ms watchdog:** The Arduino firmware automatically stops all motors if it doesn't receive a command within 200ms. This is a critical safety feature — if the V2N board crashes, hangs, or loses the serial connection, the robot stops moving within 200ms.

### 6f. Mecanum Wheel Kinematics

**Inverse Kinematics** (velocity → wheel speeds):

```
Given: vx (forward m/s), vy (strafe m/s), wz (rotation rad/s)

           ┌───┐     ┌───┐
    FL =   │vx │  -  │vy │  -  wz × L
           └───┘     └───┘
           ┌───┐     ┌───┐
    FR =   │vx │  +  │vy │  +  wz × L
           └───┘     └───┘
           ┌───┐     ┌───┐
    RL =   │vx │  +  │vy │  -  wz × L
           └───┘     └───┘
           ┌───┐     ┌───┐
    RR =   │vx │  -  │vy │  +  wz × L
           └───┘     └───┘

Where L = (wheelbase + track_width) / 2 = (0.19 + 0.21) / 2 = 0.20m
```

**Forward Kinematics** (wheel speeds → velocity):

```
Given: wFL, wFR, wRL, wRR (rad/s per wheel)
R = wheel radius = 0.04m

vx = R/4 × (wFL + wFR + wRL + wRR)      ← all wheels contribute to forward
vy = R/4 × (-wFL + wFR + wRL - wRR)      ← diagonal rollers create strafe
wz = R/(4L) × (-wFL + wFR - wRL + wRR)   ← differential creates rotation
```

---

## 7. How All Parts Connect

### 7a. The Three Threads

The application runs three threads, each with a distinct responsibility:

```
┌──────────────────────────────────────────────────────────────────────┐
│                          MAIN THREAD (GTK)                           │
│                                                                      │
│  Purpose: User interface rendering and interaction                   │
│                                                                      │
│  Gtk.main() event loop:                                              │
│  ├── 30fps timer → read all state → redraw map + camera panels      │
│  ├── Button clicks → state.nav.request_go() / request_stop()        │
│  ├── Keyboard shortcuts → G=go, S/Space=stop, Q/Esc=quit            │
│  └── Settings window → update navigator parameters via sliders      │
│                                                                      │
│  READS FROM: state.sensors, state.detection, state.nav               │
│  WRITES TO:  state.nav (go/stop requests only)                       │
└──────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────┐
│                          ROS THREAD                                   │
│                                                                      │
│  Purpose: Sensor processing, navigation decisions, motor commands    │
│                                                                      │
│  rclpy.spin_once() loop:                                             │
│  ├── /map callback     → state.sensors.set_map()                    │
│  ├── /scan callback    → state.sensors.set_laser() + set_pose()     │
│  └── control_loop 20Hz → read detections → Navigator → /cmd_vel     │
│                                                                      │
│  READS FROM: state.detection (for navigation decisions)              │
│  WRITES TO:  state.sensors (map, pose, laser)                        │
│              state.nav (nav_state, obstacles, cmd_vel)                │
│              /cmd_vel topic (motor commands)                          │
└──────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────┐
│                        CAMERA THREAD                                  │
│                                                                      │
│  Purpose: Camera capture and AI object detection                     │
│                                                                      │
│  Continuous loop:                                                    │
│  ├── cap.read()         → grab frame (~30fps)                       │
│  ├── executor.submit()  → async YOLO detection (~0.5-2fps)          │
│  └── set_camera()       → store frame + detections in state          │
│                                                                      │
│  READS FROM: state.detection (tunable params: interval, expiry)      │
│  WRITES TO:  state.detection (frames, detections, info)              │
└──────────────────────────────────────────────────────────────────────┘
```

**Why three threads?**

- The **GTK main loop** must run on the main thread (GTK3 requirement).
- **ROS2 spin** needs its own thread because `spin_once()` blocks waiting for callbacks.
- **Camera capture** needs its own thread because `cap.read()` blocks on the V4L2 driver, and AI inference is slow.

All three threads communicate through **SharedState** — a thread-safe data store with locks.

### 7b. Shared State — Thread-Safe Glue

```
SharedState (singleton — one instance shared by all threads)
│
├── sensors: SensorStore ─────────────────────────────────────────┐
│   ├── map_img + map_info           Written by: ROS thread       │
│   │                                Read by: GUI (map panel)     │
│   ├── robot_pose (x, y, θ)        Written by: ROS thread       │
│   │                                Read by: GUI + Navigator     │
│   ├── laser_points [(x,y), ...]   Written by: ROS thread       │
│   │                                Read by: GUI + Navigator     │
│   └── raw_scan (LaserScan msg)    Written by: ROS thread       │
│                                    Read by: Navigator (fusion)  │
│                                                                 │
├── detection: DetectionStore ────────────────────────────────────┤
│   ├── camera_frame (RGB numpy)     Written by: Camera thread    │
│   │                                Read by: GUI (camera panel)  │
│   ├── detections [dicts]           Written by: Camera thread    │
│   │                                Read by: ROS (control loop)  │
│   └── tunable params               Written by: GUI (settings)   │
│       (interval, expiry,           Read by: Camera thread       │
│        confidence threshold)                                    │
│                                                                 │
├── nav: NavStore ────────────────────────────────────────────────┤
│   ├── nav_state + nav_target       Written by: ROS (navigator)  │
│   │                                Read by: GUI (status bar)    │
│   ├── go/stop requests             Written by: GUI (buttons)    │
│   │                                Read by: ROS (control loop)  │
│   ├── obstacle info                Written by: ROS (navigator)  │
│   │                                Read by: GUI (status bar)    │
│   ├── cmd_vel (vx, vy, wz)        Written by: ROS (navigator)  │
│   │                                Read by: GUI (speed display) │
│   └── nav_target_map (x, y)       Written by: ROS (navigator)  │
│                                    Read by: GUI (map panel)     │
│                                                                 │
└── Thread safety: RLock with 0.1s timeout per store              │
    (never blocks forever — returns defaults on timeout)          │
```

**Why RLock with timeout?**

- A normal `Lock` would deadlock if one thread crashes while holding it.
- `RLock` allows the **same thread** to acquire the lock multiple times (reentrant).
- The **0.1s timeout** ensures that if a thread dies holding the lock, other threads recover gracefully by using default values instead of blocking forever.

### 7c. ROS2 Topic Map

```
                        ┌─────────────────────┐
                        │   rplidar_node       │
                        │   (LiDAR driver)     │
                        └────────┬────────────┘
                                 │ /scan (LaserScan)
                                 │
                    ┌────────────┼────────────────┐
                    ▼            ▼                 ▼
           ┌──────────────┐  ┌───────────┐  ┌──────────────┐
           │ cartographer  │  │ main_gui  │  │ (any other   │
           │   (SLAM)      │  │  _node    │  │  subscriber) │
           └──────┬───────┘  └───────────┘  └──────────────┘
                  │
        /map (OccupancyGrid)
        TF: map → odom
                  │
                  ▼
           ┌──────────────┐
           │  main_gui    │
           │   _node      │
           └──────────────┘

  ┌─────────────────┐     /arduino/odom_raw      ┌──────────────┐
  │ arduino_driver  │──────(JSON String)─────────►│ odometry_    │
  │   _node         │                             │   node       │
  └────────┬────────┘                             └──────┬───────┘
           │                                              │
  /cmd_vel │ (Twist)                             /odom    │ (Odometry)
           │                                     TF: odom → base_link
           │                                              │
    ┌──────┴──────┐                               ┌──────┴───────┐
    │ main_gui    │                               │ cartographer  │
    │  _node      │                               │              │
    │ (navigator) │                               └──────────────┘
    └─────────────┘

  ┌───────────────────┐
  │ robot_state_      │    TF: base_link → laser
  │   publisher       │    TF: base_link → camera_link
  │ (URDF frames)     │    TF: base_link → wheels
  └───────────────────┘
```

### 7d. Complete Data Flow — One Navigation Cycle

Here is what happens in one 50ms cycle when the robot is navigating toward a pin:

```
Time 0ms:   LiDAR completes a 360° scan
            rplidar_node publishes /scan with ~360 range values

Time 2ms:   Arduino reads 4 wheel encoders
            arduino_driver_node reads encoder counts via serial
            Publishes /arduino/odom_raw as JSON string

Time 5ms:   odometry_node receives odom_raw
            Computes body velocities using mecanum FK
            Integrates position (x += vx·cos(θ)·dt ...)
            Publishes /odom and broadcasts TF odom→base_link

Time 8ms:   Cartographer receives /scan + /odom
            Matches new scan against existing map (scan matching)
            Updates map with new observations
            Publishes corrected TF map→odom

Time 10ms:  Camera thread grabs a new frame
            If a detection finished: update cached_detections
            If time for new detection: submit async YOLO inference
            Store frame + detections in state.detection

Time 15ms:  ros_node.py scan_cb fires:
            - Converts /scan ranges to (x,y) points → state.sensors.set_laser()
            - Looks up TF map→base_link → state.sensors.set_robot_pose()

Time 20ms:  ros_node.py map_cb fires:
            - Converts OccupancyGrid to colored image → state.sensors.set_map()

Time 25ms:  ros_node.py control_loop fires:
            - Reads state.detection.get_camera() → gets latest detections
            - find_best_target() → closest pin at 0.85m, 5° right
            - navigator.navigate_to_target(target)
              ├── LiDAR fusion: check lidar distance at 5° → 0.82m (use this)
              ├── check_obstacles(): no obstacle ahead
              ├── Compute velocity: x=0.14, y=-0.01, wz=0.04
              ├── _publish_cmd(): NaN guard → speed cap → publish
              └── /cmd_vel published

Time 28ms:  arduino_driver_node receives /cmd_vel
            Converts m/s to PWM: VEL,82,-6,23
            Sends serial command to Arduino

Time 30ms:  Arduino sets PWM on PCA9685 motor shield
            All 4 motors adjust speed → robot moves toward pin

Time 33ms:  GTK main thread timer fires:
            - state.nav.get_gui_snapshot() → read nav state
            - Update status bar: "● NAVIGATING | Target: 0.82m | 0.14 m/s"
            - queue_draw() → triggers on_draw():
              ├── draw_map_panel(): SLAM map + robot dot + laser + target diamond
              └── draw_camera_panel(): camera frame + crosshair + state badge

Time 50ms:  Next control_loop iteration begins...
```

---

## 8. The GUI

The GUI is a fullscreen GTK3 window rendered on the robot's MIPI-DSI display via Weston/Wayland.

```
┌──────────────────────────────────────────────────────────────────┐
│  V2N Robot Control                  SLAM + Camera + Navigation   │
├──────────────────────────────┬───────────────────────────────────┤
│                              │                    ┌───────────┐  │
│       SLAM Map               │   Camera Feed      │NAVIGATING │  │
│                              │                    └───────────┘  │
│       ·····•                 │   ┌──────────────┐                │
│       ·   /●                 │   │    ◎────┐    │   v=0.14 m/s  │
│       · free  wall           │   │    │Pin │    │   ω=2°/s      │
│       ·    ◇ target          │   │    └────┘    │                │
│       ·                      │   │   0.82m  5°  │                │
│                              │   └──────────────┘                │
│   Robot: (1.2, 0.5) 45°     │   Detection: Pin 0.82m, 5°        │
├──────────────────────────────┴───────────────────────────────────┤
│ [GO TO TARGET]  [STOP]    ● NAVIGATING | 0.82m | 0.14 m/s       │
│                                            [SETTINGS]  [QUIT]    │
└──────────────────────────────────────────────────────────────────┘
```

### Left Panel: SLAM Map

| Element | Color | Meaning |
|---------|-------|---------|
| Dark gray cells | (30,30,30) | Unknown area (never scanned) |
| Medium gray cells | (50,50,50) | Free space (no obstacle) |
| Yellow cells | (0,200,255) | Occupied (walls, objects) |
| Green circle + arrow | (0,255,0) | Robot position and heading |
| Red dots | (0,0,255) | Current LiDAR scan points |
| Pink diamond + line | (255,0,255) | Navigation target and path |
| Gray grid lines | (60,60,60) | 1-meter reference grid |

### Right Panel: Camera Feed

| Element | Meaning |
|---------|---------|
| Camera image | Live 640×480 feed |
| Green bounding boxes | Fresh detections (<0.3s old) |
| Yellow bounding boxes | Cached detections (0.3-1.5s old) |
| Green crosshair circle | Aimed at closest pin |
| Distance/angle label | "0.82m 5°" next to crosshair |
| State badge (top-right) | Current nav state with color |
| Speed indicator (bottom-left) | "v=0.14 m/s ω=2°/s" |

### Status Bar

Color-coded with Pango markup:

- 🟢 **NAVIGATING** — Green, shows target distance + speed
- 🟡 **SEARCHING** — Yellow, shows search time + time since target lost
- 🟠 **BLIND APPROACH** — Orange, shows dead-reckoning status
- 🔵 **ARRIVED** — Blue, "at target!"
- ⚪ **IDLE** — Gray, "Press GO to start"
- 🔴 **OBS** — Red obstacle warning with distance

### Settings Window (5 tabs)

| Tab | Parameters |
|-----|-----------|
| **Navigation** | Linear speed (0.05-0.30 m/s), Min speed, Angular speed, Approach distance, Lost timeout, Search timeout, Search angular speed |
| **Blind Approach** | Entry distance, Approach speed, Timeout, LiDAR stop distance, Arrival margin |
| **Detection** | Confidence threshold (0.10-0.90), Detection interval (0.5-5.0s), Detection expiry (0.5-5.0s) |
| **Obstacle** | Stop distance, Slowdown distance, Robot half-width |
| **Calibration** | Distance calibration (place pin, enter distance, press CALIBRATE), Motor test (6 direction buttons), Odometry reset |

All slider changes take effect **immediately** — they write directly to the Navigator object's attributes.

---

## 9. Project File Structure

```
bowling_target_nav/
│
├── nodes/
│   └── main_gui.py              # Entry point (~135 lines)
│                                 # Starts 3 threads, runs GTK main loop
│
├── state/                        # Thread-safe shared data
│   ├── __init__.py              # Singleton: state = SharedState()
│   ├── shared_state.py          # Facade: composes stores + lifecycle
│   ├── sensor_store.py          # Map, pose, laser data
│   ├── detection_store.py       # Camera frames, detections, params
│   └── nav_store.py             # Nav state, commands, obstacles
│
├── nav/                          # Navigation algorithms
│   ├── navigator.py             # Direct nav, blind approach, obstacles
│   └── target_selector.py      # find_best_target() — closest pin
│
├── threads/                      # Worker threads
│   ├── ros_node.py              # ROS2 node + 20Hz control loop
│   └── camera_worker.py         # Camera capture + async detection
│
├── gui/                          # GTK3 user interface
│   ├── display.py               # Wayland/X11 auto-detection
│   ├── theme.py                 # Dark theme CSS
│   ├── main_window.py           # Fullscreen window + controls
│   ├── settings_window.py       # 5-tab parameter tuning
│   └── panels/
│       ├── map_panel.py         # SLAM map rendering
│       └── camera_panel.py      # Camera + detection overlays
│
├── detectors/                    # AI object detection
│   ├── base.py                  # DetectorBase ABC + Detection dataclass
│   ├── yolo_onnx_detector.py   # ONNX Runtime CPU inference
│   └── drp_binary_detector.py  # DRP-AI hardware pipe protocol
│
├── hardware/                     # Hardware abstraction
│   ├── arduino.py               # Motor protocol + mecanum kinematics
│   ├── arduino_bridge.py        # Serial communication + reconnect
│   ├── camera.py                # Camera capture abstraction
│   └── lidar.py                 # LiDAR abstraction
│
├── utils/
│   └── distance_estimator.py    # Bbox → distance + angle
│
├── config/
│   └── cartographer.lua         # SLAM configuration
│
├── launch/                       # ROS2 launch files
│   ├── bringup.launch.py       # Hardware drivers
│   ├── mapping.launch.py       # Cartographer SLAM
│   └── ...                      # Navigation, recording, etc.
│
├── urdf/
│   └── v2n_robot.urdf           # Robot frame definitions
│
├── scripts/
│   ├── robot_autostart.sh       # Boot script
│   ├── robot.service            # systemd unit
│   └── ...                      # Setup, utilities
│
└── models/
    ├── bowling_yolov8m.onnx     # YOLO model (float32)
    └── bowling_yolov8m_int8.onnx # YOLO model (quantized, faster)
```

---

## 10. Design Patterns Used

| Pattern | Where | Why |
|---------|-------|-----|
| **Singleton** | `state/__init__.py` exports one `SharedState` instance | All 3 threads share the same state object |
| **Facade** | `SharedState` composes 3 domain stores | Clean API: `state.sensors.get_map()` instead of raw locks |
| **Strategy** | `DetectorBase` → `YoloOnnxDetector` / `DrpBinaryDetector` | Swap detection backend without changing camera code |
| **Observer** | ROS2 pub/sub (`/scan`, `/map`, `/cmd_vel`) | Decoupled sensor producers and consumers |
| **Template Method** | `DetectorBase.detect()` calls `_detect_impl()` | Base class handles timing/stats, subclass handles inference |
| **Factory** | `create_arduino()`, `create_camera()`, `create_lidar()` | Real or mock hardware based on configuration |
| **Adapter** | `ArduinoBridge` wraps raw serial into ROS-friendly interface | Hides serial protocol behind clean API |

---

## 11. Safety Mechanisms

| Mechanism | Location | What It Prevents |
|-----------|----------|------------------|
| **200ms watchdog** | Arduino firmware | Motors stop if V2N board crashes |
| **NaN/Inf guard** | `navigator._publish_cmd()` | Corrupted sensor data doesn't become motor commands |
| **Speed cap** | `navigator._cap_speed()` | Holonomic velocity can't exceed linear_speed limit |
| **Pose staleness check** | `navigator.enter_blind_approach()` | Rejects default (0,0,0) pose that would navigate to wrong place |
| **Lock timeout (0.1s)** | All state stores | One crashed thread can't deadlock others |
| **Signal handler** | `main_gui.py signal_handler()` | SIGINT/SIGTERM schedules clean GTK quit (not direct call) |
| **Camera reconnect** | `camera_worker.py` | Automatic retry after 30 consecutive read failures |
| **Motor test timer cleanup** | `settings_window._on_delete()` | Closing settings mid-test cancels timer and stops motors |
| **ARRIVED is terminal** | `ros_node.py control_loop()` | Robot stops and stays stopped after arriving — user must press GO |
| **Arrival confirmation** | `navigator._check_arrival()` | Requires 0.3s of sustained close readings — prevents single noisy frame from causing false arrival |
| **Arrival hysteresis** | `navigator._check_arrival()` | Timer only resets when distance > 1.5× threshold — prevents oscillation near boundary |
| **Bbox clipping detection** | `camera_worker.detect_objects()` | Flags when pin bbox hits frame edge — triggers early blind approach instead of using bad distance |
| **Detection expiry** | `camera_worker.py` + `ros_node.py` | Stale detections (>1.5s old) are discarded, not acted on |
| **Minimum speed enforcement** | `navigator.direct_navigate()` | Prevents commands below motor dead zone that cause buzzing |
| **Obstacle emergency stop** | `navigator.direct_navigate()` | Zero velocity if obstacle < approach_distance with stuck timer for backup |
