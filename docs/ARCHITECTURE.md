# System Architecture

> Block diagrams, design patterns, and architectural decisions for the V2N Bowling Robot.

---

## Table of Contents

1. [High-Level Overview](#1-high-level-overview)
2. [Three-Thread Architecture](#2-three-thread-architecture)
3. [Module Dependency Diagram](#3-module-dependency-diagram)
4. [Shared State Architecture](#4-shared-state-architecture)
5. [Detection Pipeline](#5-detection-pipeline)
6. [Navigation Engine](#6-navigation-engine)
7. [ROS2 Integration](#7-ros2-integration)
8. [Design Patterns](#8-design-patterns)
9. [Safety Architecture](#9-safety-architecture)
10. [Data Flow](#10-data-flow)

---

## 1. High-Level Overview

The system is a self-contained ROS2 application that detects bowling pins with YOLO AI, maps the environment with SLAM, and navigates a mecanum-wheeled robot to the target.

```
┌─────────────────────────────────────────────────────────────────────┐
│                        V2N ROBOT SYSTEM                             │
│                                                                     │
│  ┌──────────┐   ┌──────────────┐   ┌───────────────────────────┐   │
│  │  SENSORS  │   │  PROCESSING  │   │        ACTUATORS          │   │
│  │           │   │              │   │                           │   │
│  │ Camera ───┼──►│ YOLO AI ─────┼──►│                           │   │
│  │ (30fps)   │   │ (Detection)  │   │                           │   │
│  │           │   │              │   │  Arduino Motor Controller  │   │
│  │ LiDAR  ───┼──►│ Cartographer ┼──►│  (4 mecanum wheels)       │   │
│  │ (10Hz)    │   │ (SLAM)       │   │                           │   │
│  │           │   │              │   │                           │   │
│  │ Encoders ─┼──►│ Navigator ───┼──►│  VEL,vx,vy,wz            │   │
│  │ (20Hz)    │   │ (20Hz loop)  │   │  via /cmd_vel             │   │
│  └──────────┘   └──────────────┘   └───────────────────────────┘   │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                    GTK3 FULLSCREEN GUI                        │   │
│  │  [SLAM Map]  [Camera+Detections]  [Status]  [Controls]      │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. Three-Thread Architecture

The application runs exactly three threads. Each has a dedicated responsibility with no overlap.

```
┌──────────────────────────────────────────────────────────────────┐
│                     APPLICATION PROCESS                           │
│                                                                  │
│  ┌─────────────────────┐                                         │
│  │  GTK MAIN THREAD    │  30fps rendering                        │
│  │  (main_gui.py)      │                                         │
│  │                     │  Reads: sensors, detection, nav          │
│  │  Gtk.main() loop    │  Writes: nav commands (go/stop)         │
│  │  ├─ Render map      │                                         │
│  │  ├─ Render camera   │                                         │
│  │  ├─ Update status   │                                         │
│  │  └─ Handle input    │                                         │
│  └────────┬────────────┘                                         │
│           │                                                      │
│           │ SharedState (RLock, 0.1s timeout)                    │
│           │                                                      │
│  ┌────────┴────────────┐  ┌────────────────────────┐             │
│  │  ROS2 THREAD        │  │  CAMERA THREAD         │             │
│  │  (ros_node.py)      │  │  (camera_worker.py)    │             │
│  │                     │  │                        │             │
│  │  rclpy.spin()       │  │  30fps capture         │             │
│  │  ├─ /map callback   │  │  ├─ cv2.VideoCapture   │             │
│  │  ├─ /scan callback  │  │  ├─ Async YOLO via     │             │
│  │  ├─ TF lookups      │  │  │  ThreadPoolExecutor  │             │
│  │  └─ 20Hz control    │  │  ├─ Draw overlays      │             │
│  │     loop (timer)    │  │  └─ Update store        │             │
│  │                     │  │                        │             │
│  │  Reads: detection   │  │  Reads: tunable params  │             │
│  │  Writes: sensors,   │  │  Writes: camera frame,  │             │
│  │    nav, /cmd_vel    │  │    detections           │             │
│  └─────────────────────┘  └────────────────────────┘             │
└──────────────────────────────────────────────────────────────────┘
```

### Thread Lifecycle

```
main() entry point
    │
    ├── Start ROS2 Thread (non-daemon)
    │     └── rclpy.init() → MainGuiNode() → rclpy.spin()
    │
    ├── Start Camera Thread (non-daemon)
    │     └── Open camera → Detection loop
    │
    ├── Wait for ROS ready (up to 5s)
    │
    ├── Gtk.init() → apply_theme()
    │
    ├── MainGUI() → Gtk.main()  ← BLOCKS HERE
    │
    └── Cleanup
          ├── state.request_shutdown()
          ├── Join threads (5s timeout)
          └── Exit
```

### Thread Safety Model

- All inter-thread communication goes through `SharedState`
- Each store uses `threading.RLock` with **0.1s acquisition timeout**
- Data is **copied on read** to minimize lock hold time
- Timeout failure returns safe defaults (empty list, zero values)
- No thread directly calls methods on another thread's objects

---

## 3. Module Dependency Diagram

```
                    ┌──────────────┐
                    │  main_gui.py │  Entry Point
                    └──────┬───────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
              ▼            ▼            ▼
       ┌────────────┐ ┌─────────┐ ┌──────────────┐
       │ ros_node.py│ │camera_  │ │ gui/         │
       │ (Thread 2) │ │worker.py│ │ main_window  │
       └─────┬──────┘ │(Thread3)│ │ settings_win │
             │         └────┬────┘ │ panels/      │
             │              │      └──────┬───────┘
             │              │             │
             ▼              ▼             │
       ┌──────────┐  ┌───────────┐       │
       │ nav/     │  │ detectors/│       │
       │navigator │  │ base.py   │       │
       │target_sel│  │ yolo_onnx │       │
       └────┬─────┘  │ drp_binary│       │
            │        └─────┬─────┘       │
            │              │             │
            └──────┬───────┘             │
                   │                     │
                   ▼                     ▼
            ┌─────────────────────────────────┐
            │         state/                   │
            │  shared_state.py  (Facade)       │
            │  ├── sensor_store.py             │
            │  ├── detection_store.py          │
            │  └── nav_store.py                │
            └──────────────────────────────────┘
                          │
                          ▼
            ┌─────────────────────────────────┐
            │         hardware/                │
            │  arduino.py  (Motor bridge)      │
            │  camera.py   (Video capture)     │
            │  lidar.py    (RPLidar bridge)    │
            └─────────────────────────────────┘
                          │
                          ▼
            ┌─────────────────────────────────┐
            │         utils/                   │
            │  distance_estimator.py           │
            └─────────────────────────────────┘
```

### Module Responsibilities

| Module | Responsibility | Active? |
|--------|---------------|---------|
| `nodes/main_gui.py` | Application entry, thread lifecycle | Yes |
| `threads/ros_node.py` | ROS2 node, callbacks, 20Hz nav loop | Yes |
| `threads/camera_worker.py` | Camera capture, async YOLO | Yes |
| `gui/` | GTK3 rendering, user input, settings | Yes |
| `nav/` | Navigation algorithms, obstacle avoidance | Yes |
| `detectors/` | YOLO ONNX + DRP-AI backends | Yes |
| `state/` | Thread-safe shared state stores | Yes |
| `hardware/` | Arduino, camera, LiDAR abstractions | Yes |
| `utils/` | Distance estimation from bbox | Yes |
| `core/` | Config, events, state machine (legacy) | No (retained for reference) |

---

## 4. Shared State Architecture

The `SharedState` class implements the **Facade** and **Singleton** patterns, composing three domain-specific stores.

```
┌─────────────────────────────────────────────────────────┐
│                  SharedState (Facade)                     │
│                                                          │
│  _shutdown_event: threading.Event                        │
│  _ros_node: MainGuiNode (set by ROS thread)             │
│  _errors: list (max 10, RLock protected)                │
│                                                          │
│  ┌──────────────────┐  ┌──────────────────┐             │
│  │  SensorStore      │  │  DetectionStore  │             │
│  │                   │  │                  │             │
│  │  _map_img (BGR)   │  │  _camera_frame   │             │
│  │  _map_info        │  │  _detections []  │             │
│  │  _map_count       │  │  _detection_info │             │
│  │  _robot_x/y/theta │  │  _detection_time │             │
│  │  _laser_points [] │  │                  │             │
│  │  _scan_count      │  │  Tunable Params: │             │
│  │  _raw_scan        │  │  _detect_interval│             │
│  │  _raw_scan_time   │  │  _detect_expiry  │             │
│  │                   │  │  _confidence_thr  │             │
│  │  RLock (0.1s)     │  │  _ref_box_height │             │
│  └──────────────────┘  │  _ref_distance    │             │
│                         │                  │             │
│  ┌──────────────────┐  │  RLock (0.1s)     │             │
│  │  NavStore         │  └──────────────────┘             │
│  │                   │                                   │
│  │  _nav_state (str) │  Writers / Readers:               │
│  │  _nav_target      │  ┌──────────────────────────┐    │
│  │  _go_requested    │  │ SensorStore:              │    │
│  │  _stop_requested  │  │   Write: ROS thread       │    │
│  │  _last_target_time│  │   Read: GUI, ROS thread   │    │
│  │  _search_start    │  │                          │    │
│  │  _obstacle_ahead  │  │ DetectionStore:           │    │
│  │  _obstacle_dist   │  │   Write: Camera thread    │    │
│  │  _nav_target_map  │  │   Read: ROS, GUI threads  │    │
│  │  _current_cmd_vel │  │                          │    │
│  │                   │  │ NavStore:                  │    │
│  │  RLock (0.1s)     │  │   Write: ROS thread       │    │
│  └──────────────────┘  │   Read: GUI thread         │    │
│                         │   User cmds: GUI → ROS     │    │
│                         └──────────────────────────┘    │
└─────────────────────────────────────────────────────────┘

Singleton access:
  from bowling_target_nav.state import state
  state.sensors.get_map()
  state.detection.get_camera()
  state.nav.request_go()
```

### Key Design Decisions

- **RLock** (reentrant) allows same thread to acquire multiple times
- **0.1s timeout** prevents deadlock if a thread crashes
- **Copy-on-read** (numpy `.copy()`, list slicing) minimizes lock hold time
- **`get_gui_snapshot()`** returns all GUI-needed nav data in a single lock acquisition
- **`threading.Event`** for shutdown (atomic, no lock needed)

---

## 5. Detection Pipeline

```
┌──────────────────────────────────────────────────────────────┐
│                    CAMERA THREAD                              │
│                                                              │
│  ┌──────────┐    ┌──────────────────┐    ┌───────────────┐  │
│  │ OpenCV   │    │ ThreadPoolExecutor│    │ DetectionStore│  │
│  │ V4L2     │──► │ (max_workers=1)  │──► │ (shared)      │  │
│  │ 640x480  │    │                  │    │               │  │
│  │ 30fps    │    │ ┌──────────────┐ │    │ frame (BGR)   │  │
│  └──────────┘    │ │ DRP-AI       │ │    │ detections[]  │  │
│                  │ │ (preferred)  │ │    │ info string   │  │
│  grab() ─────►  │ │ ~30-50ms     │ │    │ fresh flag    │  │
│  read()         │ ├──────────────┤ │    └───────────────┘  │
│  submit ─────►  │ │ ONNX CPU     │ │                       │
│  (rate-limited) │ │ (fallback)   │ │                       │
│                  │ │ ~100-500ms   │ │                       │
│  draw overlays  │ └──────────────┘ │                       │
│  ├─ bbox rect   └──────────────────┘                       │
│  ├─ distance                                                │
│  └─ FPS counter                                             │
└──────────────────────────────────────────────────────────────┘

Detection Flow:
  1. Camera grabs frame (non-blocking)
  2. Check if async future is done → cache results
  3. Expire old detections (> detect_expiry seconds)
  4. Submit new detection if rate allows:
     - DRP-AI: every detect_interval (default 2.0s)
     - ONNX: every frame (when no pending future)
  5. Draw cached detections on display frame
  6. Store in DetectionStore with fresh flag
```

### Strategy Pattern for Detectors

```
           DetectorBase (ABC)
           ├── name (abstract property)
           ├── supported_classes (abstract property)
           ├── _load_model() (abstract)
           ├── _detect_impl(frame) (abstract)
           │
           ├── initialize() → bool
           ├── detect(frame) → DetectionResult     ← Template Method
           │     ├── validate frame
           │     ├── call _detect_impl()
           │     ├── normalize centers
           │     └── update statistics
           ├── detect_target(frame) → Detection
           └── get_stats() → dict
                │
      ┌─────────┴──────────┐
      ▼                    ▼
YoloOnnxDetector    DrpBinaryDetector
├── ONNX Runtime    ├── C++ subprocess
├── CPU inference   ├── stdin/stdout pipe
├── YOLOv5/v8       ├── DRP-AI hardware
├── NMS + filtering ├── JSON response
└── Auto model find └── 10-20x faster
```

---

## 6. Navigation Engine

```
┌────────────────────────────────────────────────────────────────┐
│                   NAVIGATION ENGINE (20Hz)                      │
│                   ros_node.py → Navigator                       │
│                                                                │
│  Input:                                                        │
│  ├── Detection (distance, angle, bbox_clipped)                 │
│  ├── LiDAR scan points (from SensorStore)                      │
│  └── Robot pose (from TF: map → base_link)                     │
│                                                                │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                   CONTROL LOOP                            │  │
│  │                                                          │  │
│  │  1. Check user commands (GO/STOP)                         │  │
│  │  2. Get latest detections                                 │  │
│  │  3. find_best_target() → closest pin                     │  │
│  │  4. Decide action based on state:                         │  │
│  │                                                          │  │
│  │  Target visible?                                          │  │
│  │  ├── YES → navigate_to_target()                          │  │
│  │  │         ├── LiDAR+Vision fusion                       │  │
│  │  │         ├── Arrival check (multi-signal + 0.3s timer) │  │
│  │  │         ├── Blind approach entry (if clipped)         │  │
│  │  │         └── direct_navigate() (holonomic + obstacles) │  │
│  │  │                                                       │  │
│  │  └── NO                                                   │  │
│  │       ├── Lost < 3s → drift forward (min speed)          │  │
│  │       ├── Lost > 3s + close → enter_blind_approach()     │  │
│  │       ├── Lost > 3s + far → search_rotate()              │  │
│  │       └── Search timeout → IDLE                           │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                │
│  Output: /cmd_vel (Twist)                                      │
│  ├── linear.x: forward/backward (-0.15..0.15 m/s)             │
│  ├── linear.y: strafe left/right (-0.15..0.15 m/s)            │
│  └── angular.z: rotation (-0.5..0.5 rad/s)                    │
└────────────────────────────────────────────────────────────────┘
```

### Obstacle Avoidance Strategy

```
LiDAR Point Classification:

                  slowdown_distance (0.5m)
                         │
    ┌────────────────────┤
    │    FRONT CORRIDOR  │
    │    |y| < 0.15m     │
    │                    │
    │  obstacle_distance │
    │    (0.25m)         │
    │       │            │
    ├───────┤    ┌───────┤
    │ LEFT  │    │ RIGHT │
    │ ZONE  │    │ ZONE  │
    │0.15-  │ R │0.15-  │
    │0.55m  │ O │0.55m  │
    │       │ B │       │
    │       │ O │       │
    │       │ T │       │
    └───────┘   └───────┘

Decision:
  min_front < 0.25m (obstacle)?
  ├── YES
  │   ├── min_front < 0.15m → STOP (stuck > 3s → backup)
  │   ├── left_free → strafe LEFT
  │   ├── right_free → strafe RIGHT
  │   └── both blocked → BACKUP
  └── NO
      ├── min_front < 0.5m → slowdown (proportional)
      └── min_front > 0.5m → full speed
```

---

## 7. ROS2 Integration

### Node Graph

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS2 NODE GRAPH                           │
│                                                                  │
│  ┌───────────────┐        /scan         ┌──────────────────┐    │
│  │  rplidar_ros  │ ────────────────────► │                  │    │
│  │  (LiDAR drv)  │                      │  Cartographer    │    │
│  └───────────────┘                      │  SLAM            │    │
│                                          │                  │    │
│  ┌───────────────┐    /arduino/odom_raw  └───────┬──────────┘    │
│  │ arduino_driver│ ────────────────────►          │              │
│  │ _node         │                      /map     │ TF:map→odom  │
│  │               │ ◄── /cmd_vel ────┐    │       │              │
│  └───────┬───────┘                  │    ▼       ▼              │
│          │                          │                           │
│     /arduino/odom_raw               │  ┌──────────────────┐    │
│          │                          │  │   MainGuiNode    │    │
│          ▼                          │  │   (ros_node.py)  │    │
│  ┌───────────────┐                  │  │                  │    │
│  │ odometry_node │                  └──┤  Publishes:      │    │
│  │               │ ──► /odom           │   /cmd_vel       │    │
│  │               │ ──► TF:odom→base    │                  │    │
│  └───────────────┘                     │  Subscribes:     │    │
│                                         │   /map, /scan   │    │
│  ┌───────────────┐                     │   /gui/command   │    │
│  │ robot_state   │ ──► TF:base→laser   │                  │    │
│  │ _publisher    │ ──► TF:base→camera  │  Timer: 20Hz    │    │
│  └───────────────┘                     │  control loop    │    │
│                                         └──────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

### TF Frame Tree

```
              map
               │
               │ (Cartographer SLAM)
               │ Corrects drift from wheel odometry
               ▼
              odom
               │
               │ (odometry_node - encoder integration)
               │ Accurate short-term, drifts long-term
               ▼
           base_link
            ┌──┴──┐
            │     │
            ▼     ▼
          laser  camera_link
                  │
                  ▼
          wheel_fl/fr/rl/rr (static, visual only)
```

### Launch File Hierarchy

```
Full System:
  autonomous.launch.py
    ├── bringup.launch.py
    │   ├── robot_state_publisher (URDF)
    │   ├── rplidar_ros (LiDAR driver)
    │   ├── arduino_driver_node (motor control)
    │   └── odometry_node (wheel odometry + TF)
    ├── mapping.launch.py
    │   ├── cartographer_node (SLAM)
    │   └── cartographer_occupancy_grid_node
    ├── vision_node (camera + YOLO)
    └── autonomous_explorer (scan-wander-approach-push)

Main GUI (standalone):
  ros2 run bowling_target_nav main_gui
    └── Starts own ROS2 node internally
        (subscribes to /map, /scan, publishes /cmd_vel)
```

---

## 8. Design Patterns

| Pattern | Where | Purpose |
|---------|-------|---------|
| **Singleton** | `state/__init__.py` — `state = SharedState()` | Single shared state instance across all threads |
| **Facade** | `SharedState` composes 3 stores | Clean API hiding internal locks and stores |
| **Strategy** | `DetectorBase` → `YoloOnnxDetector`, `DrpBinaryDetector` | Swap detection backend without changing consumers |
| **Template Method** | `DetectorBase.detect()` calls `_detect_impl()` | Base handles timing/stats, subclass handles inference |
| **Factory** | `create_arduino()`, `create_camera()`, `create_lidar()` | Create real or mock hardware based on config |
| **Adapter** | `ArduinoBridge` wraps pyserial | Hide serial protocol details |
| **Observer** | ROS2 pub/sub (`/scan`, `/map`, `/cmd_vel`) | Decouple sensors from consumers |
| **State** | NavStore state strings + Navigator methods | Navigation behavior changes with state |

### Why These Patterns?

- **Singleton + Facade**: Three threads need to share data. A single facade with internal locks makes the API simple (`state.sensors.get_map()`) while ensuring thread safety.
- **Strategy + Factory**: The robot runs on V2N (DRP-AI available) or PC (ONNX only). Strategy lets the detection backend be swapped at runtime. Factory creates mock hardware for testing.
- **Template Method**: All detectors need timing, statistics, and error handling. Only the inference logic differs between backends.

---

## 9. Safety Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                     SAFETY LAYERS                               │
│                                                                │
│  Layer 1: HARDWARE                                              │
│  ├── Arduino 200ms watchdog (motors stop if no command)        │
│  └── PWM clamping [-255, 255]                                  │
│                                                                │
│  Layer 2: COMMAND SANITIZATION                                  │
│  ├── NaN/Inf guard (_clamp_twist) — garbage → zero             │
│  ├── Speed cap (_cap_speed) — magnitude limited                │
│  └── Min speed enforcement — overcome motor dead zone          │
│                                                                │
│  Layer 3: NAVIGATION SAFETY                                     │
│  ├── Obstacle emergency stop (< 0.25m)                         │
│  ├── Obstacle slowdown (0.25-0.5m, proportional)               │
│  ├── Stuck detection (3s at obstacle → backup)                 │
│  ├── Heading divergence check (> 45° → abort blind approach)   │
│  ├── Pose staleness rejection (default 0,0,0 → reject)        │
│  └── ARRIVED is terminal (requires user GO to restart)         │
│                                                                │
│  Layer 4: TEMPORAL FILTERS                                      │
│  ├── Arrival confirmation (0.3s sustained close signal)        │
│  ├── Arrival hysteresis (1.5x band prevents oscillation)       │
│  ├── Detection expiry (1.5s stale → discard)                  │
│  ├── Lost timeout (3s before blind approach or search)         │
│  ├── Search timeout (30s → give up)                            │
│  └── Blind approach timeout (8s → abort)                       │
│                                                                │
│  Layer 5: THREAD SAFETY                                         │
│  ├── RLock with 0.1s timeout (no deadlock)                     │
│  ├── Copy-on-read (no stale references)                        │
│  ├── Graceful degradation on lock timeout                      │
│  └── Signal handler via GLib.idle_add (safe GTK quit)          │
│                                                                │
│  Layer 6: SYSTEM                                                │
│  ├── Camera reconnect (30 failures → re-open)                  │
│  ├── Arduino auto-reconnect (exponential backoff)              │
│  ├── atexit cleanup handler                                    │
│  └── SIGINT/SIGTERM signal handlers                            │
└────────────────────────────────────────────────────────────────┘
```

---

## 10. Data Flow

### One Complete Navigation Cycle (50ms)

```
   ┌─ Camera Thread ────────────────────────────────┐
   │                                                │
   │  1. Capture frame (30fps)                       │
   │  2. Submit to YOLO (async, rate-limited)        │
   │  3. Get cached detections                       │
   │  4. Estimate distance + angle per detection    │
   │  5. Store → DetectionStore                      │
   │                                                │
   └────────────────────┬───────────────────────────┘
                        │
                        │ detections, camera frame
                        ▼
   ┌─ ROS2 Thread ──────────────────────────────────┐
   │                                                │
   │  6. /scan callback → SensorStore.set_laser()    │
   │  7. /map callback → SensorStore.set_map()       │
   │  8. TF lookup → SensorStore.set_robot_pose()    │
   │                                                │
   │  9. Control loop (20Hz timer):                  │
   │     a. Read user commands (GO/STOP)             │
   │     b. Read DetectionStore.get_camera()         │
   │     c. find_best_target() → closest pin        │
   │     d. Navigator.navigate_to_target()           │
   │        - LiDAR+Vision fusion                    │
   │        - Obstacle check                         │
   │        - Generate Twist command                 │
   │     e. Publish /cmd_vel                         │
   │     f. Update NavStore                          │
   │                                                │
   └────────────────────┬───────────────────────────┘
                        │
                        │ nav_state, cmd_vel, obstacles
                        ▼
   ┌─ GTK Thread ───────────────────────────────────┐
   │                                                │
   │  10. 30fps timer tick:                          │
   │      a. Read SensorStore → map, pose, laser    │
   │      b. Read DetectionStore → frame, detections│
   │      c. Read NavStore → state, speed, target   │
   │      d. Render map panel (robot, laser, target)│
   │      e. Render camera panel (frame, boxes)     │
   │      f. Update status label (color-coded)      │
   │                                                │
   └────────────────────────────────────────────────┘
```

### Sensor Fusion Detail

```
Vision Detection                    LiDAR Scan
(from camera thread)                (from /scan callback)
     │                                   │
     ├── distance (bbox height method)   ├── get_lidar_distance_at_angle()
     ├── angle (pixel offset → radians)  │   (query range at vision angle)
     └── bbox_clipped (near frame edge)  │   (±0.15 rad window, min valid)
          │                              │
          └──────────┬───────────────────┘
                     │
                     ▼
              ┌─────────────┐
              │   FUSION    │
              │             │
              │ if lidar < ∞│
              │   use lidar │
              │ else        │
              │   use vision│
              └──────┬──────┘
                     │
                     ▼
              fused distance
              (used for navigation)
```
