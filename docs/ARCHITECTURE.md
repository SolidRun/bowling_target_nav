# Architecture

## 1. Overview

target_nav is a ROS2 package that detects targets with DRP-AI hardware-accelerated inference and navigates a mecanum-wheeled robot to the nearest target using odometry-only localization.

**Hardware:** Renesas RZ/V2N SoC (4x Cortex-A55, DRP-AI accelerator), RPLidar A1, USB camera, Arduino-controlled mecanum drivetrain.

---

## 2. Project Structure

```
target_nav/
├── config/          # Configuration files
├── launch/          # ROS2 launch files (bringup)
├── urdf/            # Robot URDF model
├── target_nav/      # Python package
│   ├── config.py    # Single source of truth for all parameters
│   ├── app/         # Entry points and workers
│   ├── nav/         # Navigation algorithms
│   ├── detectors/   # DRP-AI detection pipeline
│   ├── hardware/    # Arduino, Camera, LiDAR drivers
│   ├── gui/         # GTK3 interface
│   ├── state/       # Thread-safe shared state
│   └── utils/       # Distance estimation, logging
├── drpai/           # C++ DRP-AI source
├── deploy/          # Pre-built DRP-AI binary + model
├── scripts/         # Shell scripts (start, sync, setup)
├── tools/           # PC-side robot controller
└── test/            # Tests
```

---

## 3. Process Architecture (4 Cores)

Each ROS2 node is pinned to a CPU core to prevent contention on the quad-core A55.

```
Core 0: GUI node       -- GTK rendering, subscribes to ROS2 topics
Core 1: nav_node       -- Navigator, publishes /cmd_vel
Core 2: camera_node    -- DRP-AI detection, publishes /detections
Core 3: arduino_node + odometry_node
```

- **Core 0 (GUI):** Runs the GTK3 fullscreen interface. Subscribes to `/detections`, `/nav_state`, `/scan`, and TF. Publishes `/nav_command` for user GO/STOP actions.
- **Core 1 (Navigator):** Runs the custom Navigator at 20 Hz. Subscribes to `/detections`, LiDAR, and TF. Publishes `/cmd_vel` and `/nav_state`.
- **Core 2 (Camera):** Captures frames, runs DRP-AI inference via the C++ binary, publishes `/detections`.
- **Core 3 (Drivers):** Arduino motor driver and wheel odometry share the last core. These are lightweight or I/O-bound.

---

## 4. Data Flow

### ROS2 Topics

| Topic | Publisher | Subscribers | Type |
|-------|-----------|-------------|------|
| `/detections` | camera_node | nav_node, GUI | Custom detection msg |
| `/nav_state` | nav_node | GUI | Navigation state info |
| `/nav_command` | GUI | nav_node | GO / STOP commands |
| `/cmd_vel` | nav_node | arduino_node | `geometry_msgs/Twist` |
| `/scan` | rplidar_ros | nav_node, GUI | `sensor_msgs/LaserScan` |
| `/odom` | odometry_node | nav_node, GUI | `nav_msgs/Odometry` |
| TF: `odom->base_link` | odometry_node | nav_node, GUI | TF2 |
| TF: `base_link->laser` | robot_state_publisher | all | TF2 (static) |

### Shared Memory (`/dev/shm`)

The C++ DRP-AI binary and Python camera node communicate through shared memory for zero-copy frame and detection transfer:

| SHM Path | Writer | Reader | Content |
|----------|--------|--------|---------|
| `/dev/shm/v2n_camera` | C++ DRP-AI | camera_node | Raw camera frames |
| `/dev/shm/v2n_detections` | C++ DRP-AI | camera_node | Detection results (JSON) |
| `/dev/shm/v2n_calibration` | camera_node | C++ DRP-AI | Runtime calibration params |

---

## 5. How Data Sync Works — Step by Step

The system has no shared memory between Python processes. Each process runs
independently with its own memory space. Data moves between them through two
mechanisms: **ROS2 topics** (Python-to-Python) and **`/dev/shm`** (C++-to-Python).

### Full Detection-to-Motor Flow

```
┌──────────────────────────────────────────────────────────────────────────┐
│ CORE 2: camera_node                                                      │
│                                                                          │
│  1. C++ DRP-AI binary captures a camera frame (640x480 BGR, 30 fps)     │
│     └─ Uses V4L2 to read from /dev/video0                               │
│     └─ Runs YOLO inference on DRP-AI hardware accelerator (not CPU)     │
│                                                                          │
│  2. C++ writes results to shared memory:                                 │
│     └─ /dev/shm/v2n_camera:     annotated frame with bounding boxes     │
│     └─ /dev/shm/v2n_detections: binary struct [confidence, bbox, ...]   │
│                                                                          │
│  3. Python camera_worker reads /dev/shm/v2n_detections (zero-copy mmap) │
│     └─ Parses binary structs into Det dataclass objects                  │
│     └─ Applies temporal tracking (DetectionTracker: min 3 consecutive   │
│        frames before accepting, smooths bbox jitter)                    │
│     └─ Estimates distance from bounding box height using calibration     │
│        formula: distance = (ref_distance * ref_box_height) / box_height │
│     └─ Estimates angle from bbox center offset vs frame center          │
│                                                                          │
│  4. camera_node publishes to /detections (std_msgs/String, JSON):       │
│     {"detections": [{"confidence": 0.82, "bbox": [120,80,200,350],      │
│       "distance": 1.35, "angle": -0.12, "class_name": "bowling-pin",        │
│       "bbox_clipped": false}], "info": "1 bowling-pin", "timestamp": ...}    │
│                                                                          │
└──────────────────────────┬───────────────────────────────────────────────┘
                           │ ROS2 topic: /detections
                           │ (DDS delivers a copy to each subscriber)
              ┌────────────┴────────────┐
              ▼                         ▼
┌─────────────────────────┐  ┌─────────────────────────┐
│ CORE 1: nav_node        │  │ CORE 0: GUI             │
│                         │  │                         │
│ 5. detections_cb()      │  │ 5'. _detections_cb()    │
│    parses JSON           │  │     stores list for     │
│    converts to Det       │  │     status bar display  │
│    objects, stores in    │  │                         │
│    local SharedState     │  │                         │
│                         │  │                         │
│ 6. control_loop (20 Hz) │  │                         │
│    └─ NavController      │  │                         │
│       reads detections   │  │                         │
│                         │  │                         │
│ 7. TF2 lookup:          │  │                         │
│    odom → base_link      │  │                         │
│    Gets robot pose       │  │                         │
│    (x, y, theta) in     │  │                         │
│    odom frame            │  │                         │
│                         │  │                         │
│ 8. Project detection     │  │                         │
│    to odom frame:        │  │                         │
│    goal_x = robot_x      │  │                         │
│      + dist * cos(θ+α)  │  │                         │
│    goal_y = robot_y      │  │                         │
│      + dist * sin(θ+α)  │  │                         │
│    (θ = robot heading,   │  │                         │
│     α = detection angle) │  │                         │
│                         │  │                         │
│ 9. Navigator computes    │  │                         │
│    velocity command:     │  │                         │
│    └─ Angle to goal →    │  │                         │
│       angular velocity   │  │                         │
│    └─ Distance to goal → │  │                         │
│       linear velocity    │  │                         │
│    └─ VFH checks LiDAR  │  │                         │
│       for obstacles →    │  │                         │
│       may adjust heading │  │                         │
│                         │  │                         │
│ 10. Publishes:           │  │                         │
│     /cmd_vel (Twist)     │  │                         │
│     /nav_state (JSON)  ──┼──┼──▶ 10'. GUI updates    │
│                         │  │       status bar,       │
│                         │  │       map overlay       │
└────────────┬────────────┘  └─────────────────────────┘
             │ ROS2 topic: /cmd_vel
             ▼
┌─────────────────────────┐
│ CORE 3: arduino_node    │
│                         │
│ 11. cmd_vel_cb()        │
│     converts Twist      │
│     (m/s) to PWM:       │
│     vx_pwm = vx * scale │
│     vy_pwm = vy * scale │
│     wz_pwm = wz * scale │
│                         │
│ 12. Sends serial:       │
│     "VEL,vx,vy,wz\n"   │
│     to Arduino via      │
│     /dev/ttyACM0        │
│                         │
│ 13. Arduino firmware    │
│     runs mecanum IK:    │
│     FL = vx - vy - wz   │
│     RL = vx + vy - wz   │
│     RR = vx - vy + wz   │
│     FR = vx + vy + wz   │
│     → 4 motors spin     │
│                         │
│ 14. Encoders feed back: │
│     Arduino sends       │
│     "ODOM,vx,vy,wz"    │
│     → odometry_node     │
│     → publishes /odom   │
│     → updated TF        │
│       odom → base_link  │
│     → nav_node uses     │
│       updated pose at   │
│       step 7 next tick  │
└─────────────────────────┘
```

### GUI Frame Display (separate path, no ROS2)

The video feed in the GUI does NOT go through ROS2 — it would be too slow
for 921 KB frames at 30 fps. Instead:

```
C++ binary writes annotated frame → /dev/shm/v2n_camera (zero-copy mmap)
                                            │
GUI reads directly via mmap ────────────────┘
  └─ _DirectShmReader in camera_panel.py
  └─ Converts BGR → Cairo surface
  └─ Renders at ~10 fps (GUI refresh rate)
```

This is the only remaining shared memory path between processes.
All other data flows through standard ROS2 topics.

### Timing Summary

| Component | Rate | Latency |
|-----------|------|---------|
| Camera capture | 30 fps | ~33 ms per frame |
| DRP-AI inference | ~10 fps | ~100 ms per inference |
| /detections publish | ~10 Hz | ~1-2 ms (local DDS) |
| Navigator control loop | 20 Hz | ~50 ms per tick |
| /cmd_vel publish | 20 Hz | ~1-2 ms (local DDS) |
| Arduino serial | 200 Hz watchdog | ~5 ms per command |
| Encoder feedback | 20 Hz | ~50 ms per update |
| GUI render | 10 fps | ~100 ms per frame |

### Why ROS2 Topics Instead of Custom IPC

The previous architecture used a custom ``IPCHub`` with shared memory,
multiprocessing locks, and sync threads (1900 lines of code). This was
replaced with standard ROS2 topics because:

1. **Zero custom code** — ROS2 DDS handles serialization, delivery, ordering.
2. **Debuggable** — run ``ros2 topic echo /detections`` from any terminal.
3. **Extensible** — adding a new subscriber is one line (``create_subscription``).
4. **Same latency** — on the same machine, DDS uses loopback shared memory (~1-2 ms).
5. **Standard** — any robotics engineer understands pub/sub topics.

---

## 6. Why Custom Navigator Instead of Nav2?

1. **Nav2 is for static goal navigation** -- it takes a fixed (x, y) coordinate and plans a path. Our use case is "find and approach a visually detected target" -- a closed-loop camera-to-motor control workflow that Nav2 does not support.

2. **Dynamic goal refinement** -- the camera continuously refines the target's map position every frame. Nav2's NavigateToPose action expects a fixed goal; canceling and resending causes path planner jitter and delays.

3. **Close-range blind zone** -- when the target is closer than the LiDAR's minimum range (0.15 m for RPLidar A1), Nav2 has no sensor data to work with. Our Navigator switches to blind approach mode, dead-reckoning from the last known map position using odometry.

4. **Camera feedback for arrival** -- our Navigator uses bounding-box size, clipping state, and LiDAR fusion to detect arrival through temporal multi-signal confirmation. Nav2 only checks odometry distance to the goal point.

5. **Resource constraints** -- the V2N board runs an ARM Cortex-A55 with limited CPU and RAM. The 8 Nav2 nodes (controller_server, planner_server, bt_navigator, smoother_server, behavior_server, waypoint_follower, velocity_smoother, collision_monitor) add ~200 MB RAM and significant CPU overhead. DRP-AI detection already consumes most available resources.

6. **We still use the ROS2 ecosystem** -- TF2 for coordinate transforms, robot_state_publisher for the URDF TF tree, odometry for localization. Only the Nav2 navigation stack was removed.

---

## 7. Navigation Pipeline

The Navigator runs a 20 Hz control loop with these stages:

```
detect --> map goal --> navigate --> refine --> blind approach --> arrive
```

1. **Detect:** Camera node publishes a detection with bounding box, confidence, estimated distance, and bearing angle.
2. **Odom goal:** Navigator projects the detection into the odom frame using the robot's current pose and the estimated distance/angle, producing a (x, y) goal in odom coordinates.
3. **Navigate:** The robot drives toward the goal using holonomic control (mecanum wheels allow simultaneous forward + strafe + rotate). VFH obstacle avoidance steers around obstacles detected by LiDAR.
4. **Refine:** Each new detection updates the goal. The robot continuously corrects its trajectory as the camera refines the target's position.
5. **Blind approach:** When the target drops below the LiDAR's minimum range (~0.15 m) or the bounding box clips the frame edge, the Navigator switches to dead-reckoning toward the last known goal using odometry only.
6. **Arrive:** Arrival is confirmed through multi-signal temporal fusion: bounding-box size exceeding threshold, bbox clipping the frame bottom, and LiDAR distance below threshold -- sustained for 0.3 s to prevent false positives.

### State Transitions

```
IDLE --> NAVIGATING --> BLIND_APPROACH --> ARRIVED
  ^          |                |               |
  |          v                v               |
  +------ SEARCHING <--------+               |
  +-------------------------------------------+
```

- **IDLE:** Waiting for user GO command.
- **NAVIGATING:** Driving toward a visible target with obstacle avoidance.
- **SEARCHING:** Target lost for > 3 s; executing a 360-degree rotation scan.
- **BLIND_APPROACH:** Target below LiDAR min range; dead-reckoning to last known position.
- **ARRIVED:** Target reached. Terminal state; requires user GO to restart.

---

## 8. Sensor Offset Model

The camera and LiDAR are mounted at different positions on the robot. This creates a discrepancy between vision-estimated distance and LiDAR-measured distance that must be accounted for.

```
        Camera (forward-facing, elevated)
           |
           |  camera_offset_x (forward from base_link)
           |
      base_link ---- LiDAR (center of robot)
```

- **Camera distance:** Estimated from bounding-box height using a reference-point calibration model. Less accurate but works at all ranges.
- **LiDAR distance:** Measured directly from the laser scan at the detection's bearing angle. Accurate but has a 0.15 m minimum range (blind zone).
- **Fusion rule:** Use LiDAR distance when available (> 0.15 m and valid reading at the target angle). Fall back to vision distance otherwise.
- **Reference points:** The distance estimator supports two reference points -- "front" (bumper) and "center" (base_link). The chosen reference affects the arrival distance threshold.
- **Blind zone:** Below ~0.15 m from the LiDAR, only vision and odometry are available. The blind approach phase handles this region.

---

## 9. Configuration

### Single Source of Truth: `config.py`

All compile-time constants live in `target_nav/config.py`:

- Navigation parameters (speeds, thresholds, timeouts)
- Hardware settings (serial ports, baud rates, camera resolution)
- Detection parameters (confidence threshold, expiry time)
- Sensor geometry (camera/LiDAR offsets)
- Process affinity (core assignments)

### Runtime Settings: `SettingsStore`

Tunable parameters that users adjust through the GUI settings panel at runtime:

- Detection confidence threshold
- Reference box height and distance (calibration)
- Navigation speed limits
- Obstacle avoidance distances
- Map rendering options

### Persistence: `calibration.json`

Runtime settings are persisted to `~/.config/target_nav/calibration.json`. Changes auto-save with a 2 s debounce. On startup, saved values override `config.py` defaults for any keys present in the JSON file.

---

## 10. Why No Standard ROS2 Packages?

| Package | Why not used | What we use instead |
|---------|-------------|-------------------|
| **Nav2** (full stack) | Designed for static-goal A-to-B navigation. Cannot handle dynamic camera-driven goals with continuous refinement. | Custom Navigator with 20 Hz camera-in-the-loop control. |
| **nav2_map_server** | No maps are used. The system uses odometry-only localization. | Odometry provides `odom->base_link` TF directly. |
| **AMCL** | Localization against a prior map. No maps are used. | Wheel odometry provides localization. |
| **move_base / DWB controller** | Generic differential-drive path following. Does not exploit mecanum holonomic motion or integrate camera feedback. | Custom VFH obstacle avoidance tuned for omnidirectional mecanum movement. |

### Packages We Do Use

| Package | Purpose |
|---------|---------|
| **robot_state_publisher** | Publishes the URDF TF tree (`base_link->laser`, `base_link->camera_link`, wheel frames). |
| **TF2** | Coordinate frame transforms used by Navigator and GUI for map-frame reasoning. |
| **rplidar_ros** | Driver for the RPLidar A1, publishes `/scan`. |
