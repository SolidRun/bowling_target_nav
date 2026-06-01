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
   -> Publishes /cmd_vel (Twist) with linear.x/y (m/s) and angular.z (rad/s)
   -> Writes nav state to NavShmWriter (/dev/shm/v2n_nav)
   -> Writes LiDAR points to LaserShmWriter (/dev/shm/v2n_laser)

4. ArduinoDriverNode (arduino_node.py, launched by bringup)
   Two command paths share one serial port, controlled by _vel_mode flag:
     VEL mode (_vel_mode=True): nav publishes /cmd_vel -> command_loop sends VEL
     Position mode (_vel_mode=False): GUI sends /arduino/cmd -> direct passthrough
   a. _cmd_vel_callback stores latest Twist + timestamp, sets _vel_mode=True
   b. 20Hz timer (_command_loop) runs every 50ms (only when _vel_mode=True):
      -> Checks timeout: if no cmd_vel for 0.5s -> sends STOP once, _vel_mode=False
      -> Converts units: m/s * 1000 = mm/s, rad/s * 1000 = mrad/s
      -> Calls bridge.send_velocity(vx_mm, vy_mm, wz_mrad)
   c. _arduino_cmd_callback sets _vel_mode=False, forwards raw command to firmware
      (FWD,100,1719 / TMOTOR,FL,100 / CALIB / STOP)
   d. ArduinoBridge writes command to USB serial (auto-detected port, e.g. /dev/ttyACM0)

5. Arduino firmware (on Arduino Mega via auto-detected USB serial port)
   a. serial_cmd.cpp parses "VEL,200,0,300" -> Command{VELOCITY, vx=200, vy=0, wz=300}
   b. robot.cpp handleCommand(VELOCITY):
      -> Mecanum::computeFromVelocity(200, 0, 300, tickRates[])
         mm/s -> per-wheel tick rates via mecanum inverse kinematics
      -> Motion::setMotorTickRates(tickRates) sets PID velocity setpoints
      -> Enables 200ms watchdog (no new VEL within 200ms -> auto-STOP)
   c. Timer1 ISR fires at 50Hz -> Motion::update():
      -> Per-motor PID loop: read encoder, compute error, adjust PWM
      -> PWM = feed-forward + Kp*error + Ki*integral (per motor)
      -> Motor::setAll(pwm) writes all 4 motors atomically
   d. motor.cpp converts PWM 0-255 -> 12-bit duty cycle (0-4095)
   e. PCA9685 (I2C at 0x60) generates 1600Hz PWM on 8 channels
   f. TB67H450 H-bridges switch battery power to motors
   g. DC motors spin through 90:1 gearbox -> mecanum wheels turn

6. Odometry feedback (20Hz, every 50ms, VEL mode only)
   a. Firmware reads 4 encoder snapshots (atomic, interrupts disabled)
   b. Computes tick deltas -> Mecanum::forwardKinematics() -> vx,vy,wz
   c. Sends "ODOM,vx_mm,vy_mm,wz_mrad\n" over serial
   d. ArduinoBridge background read thread receives line
   e. ArduinoDriverNode publishes JSON on /arduino/odom_raw
   f. OdometryNode converts mm/s -> m/s, integrates pose -> publishes /odom
   g. TF: odom -> base_link updated
   h. Nav process reads /odom -> computes new error -> new Twist -> loop
   Note: ENC telemetry (raw ticks, sent during IDLE/test modes) is ignored
   by the ROS2 side. Only ODOM is used for navigation odometry.

7. GUI (Core 0) SHM poll thread reads at 20 Hz:
   -> NavShmReader -> SharedState.nav (status bar, map overlay)
   -> LaserShmReader -> SharedState.sensors (radar view)
   -> DetShmReader -> SharedState.detection (camera panel info bar)
   -> /dev/shm/v2n_camera -> camera panel (raw BGR via mmap)
   -> GUI commands -> CmdRingBuffer.push() -> Nav polls at 20 Hz
```

### Safety Layers (any one stops the robot independently)

| Layer | Where | Timeout | Trigger |
|-------|-------|---------|---------|
| Nav node | Python (Core 1) | — | Stops publishing /cmd_vel when arrived |
| ArduinoDriverNode | Python (arduino_node.py) | 0.5s | No cmd_vel received -> sends STOP |
| Firmware watchdog | Arduino (robot.cpp) | 200ms | No VEL command received -> auto-STOP |
| Firmware stall detect | Arduino (motion.cpp) | 5s | Encoder not progressing -> STOP + ERROR |
| Firmware move timeout | Arduino (robot.cpp) | 30s | Position move taking too long -> STOP |

### Timing Summary

| Component | Rate | Latency |
|-----------|------|---------|
| Camera capture | 30 fps | ~33 ms per frame |
| DRP-AI inference | ~10 fps | ~100 ms per inference |
| Struct SHM write/read | 20 Hz | <1 ms (mmap, no serialization) |
| Navigator control loop | 20 Hz | ~50 ms per tick |
| Arduino serial | 20 Hz (200ms watchdog) | ~5 ms per command |
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

## 7. Navigation, Detection Filtering, and Algorithms

See [NAVIGATION.md](NAVIGATION.md) for the complete navigation reference:

- State machine (IDLE → NAVIGATING → SEARCHING → SPIRAL_SEARCH → BLIND_APPROACH → ARRIVED)
- VFH obstacle avoidance algorithm
- Detection filtering pipeline (3-layer false positive rejection)
- Kalman-filtered target tracking with LiDAR fusion
- Speed ramp and arrival detection
- Sensor offset model and distance fusion
- Troubleshooting guide

---

## 8. Configuration

---

## 8. Why No Standard ROS2 Packages?

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
