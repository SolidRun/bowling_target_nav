# Architecture

## 1. Overview

target_nav is a ROS2 package that detects bowling-pins with DRP-AI hardware-accelerated inference and navigates a mecanum-wheeled robot to the nearest target using odometry-only localization.

**Target:** bowling-pin (0.28m height x 0.08m width). Distance estimation via Python pinhole model (not C++).

**Hardware:** Renesas RZ/V2N SoC (4x Cortex-A55, DRP-AI accelerator), RPLidar A1, USB camera, Arduino-controlled mecanum drivetrain.

---

## 2. Project Structure

```
target_nav/
├── launch/          # ROS2 launch files (bringup, record)
├── urdf/            # Robot URDF model
├── target_nav/      # Python package
│   ├── config.py    # Single source of truth for all parameters
│   ├── app/         # Entry points and workers
│   ├── nav/         # Navigation algorithms (VFH, blind, search, arrival)
│   ├── detectors/   # DRP-AI detection pipeline
│   ├── hardware/    # Arduino, Camera, LiDAR drivers
│   ├── gui/         # GTK3 interface (panels/, settings_tabs/)
│   ├── state/       # Thread-safe shared state (stores + dispatcher)
│   ├── ipc/         # Lock-free struct SHM readers/writers
│   └── utils/       # Distance estimation, logging
├── drpai/           # C++ DRP-AI source
├── deploy/          # Pre-built DRP-AI binary + model
├── scripts/         # Shell scripts (start, sync, setup)
└── test/            # Tests (unit/, utils/)
```

---

## 3. Process Architecture (3-process, lock-free struct SHM)

3 Python processes pinned to CPU cores with lock-free SPSC struct SHM IPC. No Bridge process, no JSON for real-time data.

```
Core 0: GUI process      -- Pure GTK, reads struct SHM directly, no ROS2
Core 1: Nav process      -- ROS2 navigation, Kalman tracker, motor commands (nice -5)
Core 2: Camera process   -- DRP-AI subprocess management, detection tracking
Core 3: DRP-AI C++ binary -- YOLO inference (pinned to isolate heavy CPU from nav)
```

- **Core 0 (GUI):** Pure GTK3 fullscreen interface. A SHM poll daemon thread reads struct SHM from Nav and Camera processes and populates a local SharedState. GUI commands (GO, STOP) are written to a CmdRingBuffer in SHM. No ROS2 in this process.
- **Core 1 (Nav, nice -5):** ROS2 node with TF2, 20 Hz control loop (NavController + Navigator), Kalman-filtered target tracker, VFH obstacle avoidance, blind approach, spiral search. Reads detections from DetShmReader, commands from CmdRingBuffer. Writes nav state via NavShmWriter and LiDAR via LaserShmWriter.
- **Core 2 (Camera):** Manages the C++ DRP-AI subprocess. Reads detection structs from C++ SHM, applies temporal tracking (DetectionTracker), computes distance via Python pinhole model, writes to DetShmWriter for both Nav and GUI.
- **Core 3 (DRP-AI C++):** The `app_yolo_cam` binary captures camera frames, runs YOLO inference on DRP-AI hardware, writes raw BGR frames to `/dev/shm/v2n_camera` and detection structs to `/dev/shm/v2n_detections`.

---

## 4. Data Flow (all struct SHM, no JSON for real-time data)

### Lock-free struct SHM channels (Python-to-Python)

| SHM Path | Writer | Reader(s) | Content | Protocol |
|----------|--------|-----------|---------|----------|
| `/dev/shm/v2n_det` | Camera (Core 2) | Nav (Core 1), GUI (Core 0) | Detection structs | SPSC, seq sandwich |
| `/dev/shm/v2n_nav` | Nav (Core 1) | GUI (Core 0) | Nav state snapshot | SPSC, seq sandwich |
| `/dev/shm/v2n_laser` | Nav (Core 1) | GUI (Core 0) | LiDAR points + pose | SPSC, seq sandwich |
| `/dev/shm/v2n_cmd` | GUI (Core 0) | Nav (Core 1) | Command ring buffer | SPSC ring, seq per slot |

### C++ DRP-AI shared memory (C++-to-Python)

| SHM Path | Writer | Reader | Content |
|----------|--------|--------|---------|
| `/dev/shm/v2n_camera` | C++ DRP-AI (Core 3) | GUI (Core 0) | Raw BGR frames |
| `/dev/shm/v2n_detections` | C++ DRP-AI (Core 3) | Camera (Core 2) | Detection binary structs |
| `/dev/shm/v2n_calibration` | Camera (Core 2) | C++ DRP-AI (Core 3) | Calibration params |

### ROS2 topics (Nav + Camera processes, backup path)

| Topic | Publisher | Subscriber | Type |
|-------|-----------|------------|------|
| `/scan` | rplidar_ros | nav_node | `sensor_msgs/LaserScan` |
| `/cmd_vel` | nav_node | arduino_node | `geometry_msgs/Twist` |
| `/arduino/cmd` | nav_node | arduino_node | `std_msgs/String` |
| `/reset_odom` | nav_node | odometry_node | `std_msgs/Empty` |
| `/nav_state` | nav_node | (backup) | `std_msgs/String` (JSON) |
| `/settings_changed` | GUI (via SHM cmd) | nav_node, camera_node | `std_msgs/String` |
| `/detections` | camera_node | (backup) | `std_msgs/String` (JSON) |
| `/detector_mode` | camera_node | (info) | `std_msgs/String` |
| `/drpai_restart` | GUI (via SHM cmd) | camera_node | `std_msgs/String` |
| TF: `odom->base_link` | odometry_node | nav_node | TF2 |
| TF: `base_link->camera_link` | nav_node (static) | — | TF2 |
| TF: `base_link->laser` | nav_node (static) | — | TF2 |

Note: The GUI has no ROS2 dependency. All GUI data comes from struct SHM.

### Data flow diagram

```
Camera (Core 2) --DetShmWriter--> Nav (Core 1) via DetShmReader [20Hz poll]
Camera (Core 2) --DetShmWriter--> GUI (Core 0) via DetShmReader [20Hz poll]
Camera (Core 2) --/dev/shm/v2n_camera--> GUI (Core 0) [raw BGR frames from C++]
Nav (Core 1) --NavShmWriter--> GUI (Core 0) via NavShmReader [20Hz poll]
Nav (Core 1) --LaserShmWriter--> GUI (Core 0) via LaserShmReader [20Hz poll]
GUI (Core 0) --CmdRingBuffer.push()--> Nav (Core 1) via CmdRingBuffer.pop() [20Hz poll]
```

---

## 5. Lock-free Protocol

Every struct SHM channel uses SPSC (Single Producer, Single Consumer) with torn-read protection:

1. Writer increments a monotonic sequence counter.
2. Writer writes `seq_start` to the buffer header.
3. Writer fills the payload via `struct.pack_into`.
4. Writer writes `seq_end = seq_start` to the buffer trailer.
5. Reader checks `seq_start == seq_end` and `seq != last_seen_seq`.
6. If torn (seq_start != seq_end), reader returns None and retries next tick.

No locks, no mutexes, no CAS. The SPSC invariant guarantees correctness.

### Full Detection-to-Motor Flow

```
1. C++ DRP-AI binary (Core 3) captures frame, runs YOLO inference
   -> Writes detection structs to /dev/shm/v2n_detections
   -> Writes raw BGR frame to /dev/shm/v2n_camera

2. Python camera_worker (Core 2) reads /dev/shm/v2n_detections
   -> Temporal tracking (DetectionTracker)
   -> Shape validation (bowling-pin width/height ratio)
   -> Python pinhole distance estimation
   -> Writes to DetShmWriter (/dev/shm/v2n_det)

3. Nav process (Core 1) polls DetShmReader at 20 Hz
   -> TargetTracker: size-distance gate, LiDAR matching, Kalman filter
   -> NavController state machine: NAVIGATING / SEARCHING / BLIND_APPROACH
   -> Navigator: VFH obstacle avoidance, holonomic velocity commands
   -> Publishes /cmd_vel (Twist)
   -> Writes nav state to NavShmWriter (/dev/shm/v2n_nav)
   -> Writes LiDAR points to LaserShmWriter (/dev/shm/v2n_laser)

4. Arduino node (launched by bringup) converts Twist to PWM
   -> Sends "VEL,vx,vy,wz\n" to Arduino firmware
   -> Firmware runs mecanum IK -> 4 motors spin
   -> Encoders feed back "ODOM,vx,vy,wz" -> odometry_node
   -> TF: odom -> base_link updated

5. GUI (Core 0) SHM poll thread reads at 20 Hz:
   -> NavShmReader -> SharedState.nav (status bar, map overlay)
   -> LaserShmReader -> SharedState.sensors (radar view)
   -> DetShmReader -> SharedState.detection (camera panel info bar)
   -> /dev/shm/v2n_camera -> camera panel (raw BGR via mmap)
   -> GUI commands -> CmdRingBuffer.push() -> Nav polls at 20 Hz
```

### Timing Summary

| Component | Rate | Latency |
|-----------|------|---------|
| Camera capture | 30 fps | ~33 ms per frame |
| DRP-AI inference | ~10 fps | ~100 ms per inference |
| Struct SHM write/read | 20 Hz | <1 ms (mmap, no serialization) |
| Navigator control loop | 20 Hz | ~50 ms per tick |
| Arduino serial | 200 Hz watchdog | ~5 ms per command |
| Encoder feedback | 20 Hz | ~50 ms per update |
| GUI render | 10 fps | ~100 ms per frame |

---

## 6. Why Custom Navigator Instead of Nav2?

1. **Nav2 is for static goal navigation** -- it takes a fixed (x, y) coordinate and plans a path. Our use case is "find and approach a visually detected bowling-pin" -- a closed-loop camera-to-motor control workflow that Nav2 does not support.

2. **Dynamic goal refinement** -- the camera continuously refines the target's map position every frame. Nav2's NavigateToPose action expects a fixed goal; canceling and resending causes path planner jitter and delays.

3. **Close-range blind zone** -- when the target is closer than the LiDAR's minimum range (0.15 m for RPLidar A1), Nav2 has no sensor data to work with. Our Navigator switches to blind approach mode, dead-reckoning from the last known map position using odometry.

4. **Camera feedback for arrival** -- our Navigator uses bounding-box size, clipping state, and LiDAR fusion to detect arrival through temporal multi-signal confirmation. Nav2 only checks odometry distance to the goal point.

5. **Resource constraints** -- the V2N board runs an ARM Cortex-A55 with limited CPU and RAM. The 8 Nav2 nodes add ~200 MB RAM and significant CPU overhead. DRP-AI detection already consumes most available resources.

6. **We still use the ROS2 ecosystem** -- TF2 for coordinate transforms, robot_state_publisher for the URDF TF tree, odometry for localization. Only the Nav2 navigation stack was removed.

---

## 7. Navigation Pipeline

The Navigator runs a 20 Hz control loop with these stages:

```
detect --> Kalman track --> map goal --> navigate --> refine --> blind approach --> arrive
```

1. **Detect:** Camera process writes detections to struct SHM. Nav process reads via DetShmReader at 20 Hz.
2. **Track:** TargetTracker validates detections (size-distance gate, shape filter), matches with LiDAR, and runs a Kalman filter for smoothed position.
3. **Odom goal:** Navigator projects the tracked target into the odom frame using the robot's current pose, producing a (x, y) goal in odom coordinates.
4. **Navigate:** The robot drives toward the goal using holonomic control (mecanum wheels allow simultaneous forward + strafe + rotate). VFH obstacle avoidance steers around obstacles detected by LiDAR.
5. **Refine:** Each new detection updates the Kalman state and the odom goal. The robot continuously corrects its trajectory.
6. **Blind approach:** When the target drops below the LiDAR's minimum range (~0.15 m) or the bounding box clips the frame edge, the Navigator switches to dead-reckoning toward the last known goal using odometry only.
7. **Arrive:** Arrival is confirmed through multi-signal temporal fusion: smallest distance from all sources stays within approach_distance for 0.3 s.

### State Transitions

```
IDLE --> NAVIGATING --> BLIND_APPROACH --> ARRIVED
  ^          |                |               |
  |          v                v               |
  +------ SEARCHING <--------+               |
  +-------------------------------------------+
```

- **IDLE:** Waiting for user GO command.
- **NAVIGATING:** Driving toward a visible bowling-pin with obstacle avoidance.
- **SEARCHING:** Target lost for > lost_timeout; 360-degree rotation scan, then spiral.
- **BLIND_APPROACH:** Target below LiDAR min range; dead-reckoning to last known position.
- **ARRIVED:** Target reached. Terminal state; requires user GO to restart.

---

## 8. Sensor Offset Model

The camera and LiDAR are mounted at different positions on the robot. This creates a discrepancy between vision-estimated distance and LiDAR-measured distance that must be accounted for.

```
        Camera (forward-facing, elevated)
           |
           |  camera_offset_x = 0.15m (forward from base_link)
           |
      base_link ---- LiDAR (offset_x = 0.12m forward)
```

- **Camera distance:** Estimated from bounding-box height using a Python pinhole model with reference-point calibration. Less accurate but works at all ranges.
- **LiDAR distance:** Measured directly from the laser scan at the detection's bearing angle. Accurate but has a 0.15 m minimum range (blind zone).
- **Fusion rule (TargetTracker):** LiDAR distance is preferred when a matching obstacle is found within the angular and distance window. Camera distance is used otherwise. The Kalman filter uses lower measurement noise for LiDAR-confirmed readings.
- **Reference points:** The distance estimator supports four reference points: "center" (base_link), "front" (bumper), "camera", and "lidar". The chosen reference affects the arrival distance threshold.
- **Blind zone:** Below ~0.15 m from the LiDAR, only vision and odometry are available. The blind approach phase handles this region.

---

## 9. Configuration

### Single Source of Truth: `config.py`

All compile-time constants live in `target_nav/config.py`:

- Navigation parameters (speeds, thresholds, timeouts)
- Hardware settings (serial ports, baud rates, camera resolution)
- Detection parameters (confidence threshold, expiry time)
- Sensor geometry (camera/LiDAR offsets)
- Target profile (bowling-pin: 0.28m height, 0.08m width)
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
| **TF2** | Coordinate frame transforms used by Navigator for odom-frame goal computation. |
| **rplidar_ros** | Driver for the RPLidar A1, publishes `/scan`. |
