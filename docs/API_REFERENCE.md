# API Reference

> Complete module, class, and function reference for the bowling_target_nav package.

---

## Table of Contents

1. [Package Overview](#1-package-overview)
2. [state/ — Thread-Safe Shared State](#2-state--thread-safe-shared-state)
3. [nav/ — Navigation Algorithms](#3-nav--navigation-algorithms)
4. [threads/ — Worker Threads](#4-threads--worker-threads)
5. [detectors/ — AI Detection Backends](#5-detectors--ai-detection-backends)
6. [gui/ — GTK3 User Interface](#6-gui--gtk3-user-interface)
7. [hardware/ — Hardware Abstractions](#7-hardware--hardware-abstractions)
8. [utils/ — Utilities](#8-utils--utilities)
9. [nodes/ — ROS2 Entry Points](#9-nodes--ros2-entry-points)
10. [Configuration Files](#10-configuration-files)

---

## 1. Package Overview

```python
import bowling_target_nav           # Package root
from bowling_target_nav.state import state  # Singleton shared state
from bowling_target_nav.nav import Navigator, find_best_target
from bowling_target_nav.detectors import DetectorBase, YoloOnnxDetector, DrpBinaryDetector
from bowling_target_nav.hardware import ArduinoBridge, CameraCapture, LidarBridge
```

---

## 2. state/ — Thread-Safe Shared State

### `SharedState` (state/shared_state.py)

Facade composing three domain-specific stores. Singleton instance exported as `state`.

```python
from bowling_target_nav.state import state
```

| Property/Method | Returns | Description |
|----------------|---------|-------------|
| `state.running` | `bool` | `True` while application is running |
| `state.request_shutdown()` | `None` | Signal all threads to stop |
| `state.sensors` | `SensorStore` | Map, pose, laser data |
| `state.detection` | `DetectionStore` | Camera, detections, tunable params |
| `state.nav` | `NavStore` | Navigation state, commands, obstacles |
| `state.add_error(source, error)` | `None` | Thread-safe error logging (max 10) |
| `state.get_errors()` | `list[str]` | Get error list (format: `"source: error"`) |
| `state.clear_errors()` | `None` | Clear error list |
| `state._ros_node` | `Node or None` | ROS2 node reference (set by ROS thread) |

---

### `SensorStore` (state/sensor_store.py)

Thread-safe container for SLAM map, robot pose, and LiDAR scan data.

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `set_map` | `(img: ndarray, info)` | `bool` | Store SLAM occupancy grid (BGR) |
| `get_map` | `()` | `(img_copy, info, count)` | Get map with change counter |
| `set_robot_pose` | `(x: float, y: float, theta: float)` | `bool` | Store pose from TF |
| `get_robot_pose` | `()` | `(x, y, theta)` | Get robot position and heading |
| `set_laser` | `(points: list[tuple])` | `bool` | Store LiDAR as (x,y) list |
| `get_laser` | `()` | `(points_copy, count)` | Get laser points with change counter |
| `set_raw_scan` | `(scan_msg: LaserScan)` | `bool` | Store raw LaserScan message |
| `get_lidar_distance_at_angle` | `(angle_rad: float, window: float=0.15)` | `float` | Query range at specific angle |

**`get_lidar_distance_at_angle`** detail:
- Converts angle to LaserScan indices
- Returns minimum valid range within `±window` radians
- Returns `float('inf')` if no valid readings or scan stale (>0.5s)

---

### `DetectionStore` (state/detection_store.py)

Thread-safe container for camera frames, detections, and tunable parameters.

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `set_camera` | `(frame, detections, info, fresh_detection=False)` | `bool` | Store camera frame + detections |
| `get_camera` | `()` | `(frame_copy, detections_copy, info, age_seconds)` | Get frame with detection age |
| `set_detect_interval` | `(val: float)` | `None` | Min seconds between detection runs |
| `get_detect_interval` | `()` | `float` | Current detection interval |
| `set_detect_expiry` | `(val: float)` | `None` | Seconds before detection expires |
| `get_detect_expiry` | `()` | `float` | Current detection expiry |
| `set_confidence_threshold` | `(val: float)` | `None` | Min YOLO confidence |
| `get_confidence_threshold` | `()` | `float` | Current threshold |
| `set_calibration` | `(box_height: float, distance: float)` | `None` | Distance calibration reference |
| `get_calibration` | `()` | `(height, distance)` | Current calibration values |

**Default values**: detect_interval=2.0s, detect_expiry=1.5s, confidence=0.35, ref_box_height=100px, ref_distance=1.0m

---

### `NavStore` (state/nav_store.py)

Thread-safe container for navigation state, commands, and obstacle info.

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `set_nav_state` | `(nav_state: str, target=None)` | `bool` | Update nav state and optional target (False if lock timeout) |
| `get_nav_state` | `()` | `(state_str, target_tuple)` | Current state and target (`"UNKNOWN", None` on lock timeout) |
| `request_go` | `()` | `None` | User presses GO |
| `request_stop` | `()` | `None` | User presses STOP |
| `check_and_clear_go` | `()` | `bool` | Atomic read + reset of go flag |
| `check_and_clear_stop` | `()` | `bool` | Atomic read + reset of stop flag |
| `update_target_seen` | `()` | `None` | Update last-seen timestamp |
| `get_time_since_target` | `()` | `float` | Seconds since target was last visible |
| `start_search` | `()` | `None` | Set state to "SEARCHING", record start time |
| `get_search_time` | `()` | `float` | Seconds spent searching |
| `set_obstacle` | `(ahead: bool, dist: float)` | `None` | Update obstacle status |
| `get_obstacle` | `()` | `(ahead: bool, dist: float)` | Get obstacle info |
| `set_nav_target_map` | `(x: float, y: float)` | `None` | Target in map frame (for GUI) |
| `clear_nav_target_map` | `()` | `None` | Clear map-frame target |
| `get_nav_target_map` | `()` | `tuple or None` | Get map-frame target |
| `set_current_cmd_vel` | `(vx, vy, wz)` | `None` | Store published velocity |
| `get_current_cmd_vel` | `()` | `(vx, vy, wz)` | Get last velocity |
| `get_gui_snapshot` | `()` | `dict` | All GUI-needed fields in one lock |

**`get_gui_snapshot`** returns:
```python
{
    'nav_state': str,
    'nav_target': tuple or None,
    'time_since_target': float,
    'search_time': float,
    'obstacle_ahead': bool,
    'obstacle_dist': float,
    'cmd_vel': (vx, vy, wz),
    'nav_target_map': tuple or None
}
```

**Navigation states**: `"IDLE"`, `"SEARCHING"`, `"NAVIGATING"`, `"BLIND_APPROACH"`, `"ARRIVED"`, `"UNKNOWN"` (lock timeout fallback)

> The GUI also handles `"ERROR"` state for display purposes (badge color defined in camera_panel.py).

---

## 3. nav/ — Navigation Algorithms

### `Navigator` (nav/navigator.py)

Core navigation class implementing holonomic drive, obstacle avoidance, and blind approach.

```python
Navigator(state, cmd_vel_pub, tf_buffer, nav_client, logger)
```

#### Constructor Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `state` | `SharedState` | Thread-safe shared state |
| `cmd_vel_pub` | `rclpy.Publisher` | Publisher for /cmd_vel |
| `tf_buffer` | `tf2_ros.Buffer` | TF2 transform buffer |
| `nav_client` | `ActionClient or None` | Optional Nav2 client |
| `logger` | `rclpy.Logger` | ROS2 logger |

#### Tunable Parameters (set via Settings GUI)

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `approach_distance` | float | 0.15 m | Arrival detection threshold |
| `linear_speed` | float | 0.15 m/s | Navigation forward speed |
| `min_linear_speed` | float | 0.10 m/s | Motor dead zone floor |
| `angular_speed` | float | 0.5 rad/s | Maximum rotation speed |
| `obstacle_distance` | float | 0.25 m | Emergency stop trigger |
| `obstacle_slowdown_distance` | float | 0.5 m | Gradual slowdown zone |
| `robot_half_width` | float | 0.15 m | LiDAR corridor width |
| `search_angular_speed` | float | 0.4 rad/s | Search rotation speed |
| `lost_timeout` | float | 3.0 s | Before blind approach/search |
| `search_timeout` | float | 30.0 s | Maximum search duration |
| `blind_approach_entry_distance` | float | 0.80 m | Activation threshold |
| `blind_approach_speed` | float | 0.10 m/s | Dead-reckon speed |
| `blind_approach_timeout` | float | 8.0 s | Max blind approach time |
| `blind_approach_lidar_stop` | float | 0.12 m | Emergency stop distance |
| `blind_approach_arrival_margin` | float | 0.10 m | Arrival tolerance |

#### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `navigate_to_target` | `(target: dict)` | `None` | Main entry: LiDAR+Vision fusion, arrival check, holonomic nav |
| `direct_navigate` | `(target_x, target_y, distance)` | `None` | Mecanum navigation with obstacle avoidance |
| `enter_blind_approach` | `()` | `bool` | Switch to dead-reckoning mode |
| `blind_approach_step` | `()` | `None` | Execute one tick of dead-reckoning |
| `search_rotate` | `()` | `None` | Rotate in place to find target |
| `stop_robot` | `()` | `None` | Emergency stop, reset timers |
| `check_obstacles` | `()` | `(min_front, left_free, right_free)` | LiDAR obstacle detection |

#### Target Dict Format

```python
target = {
    'distance': 0.45,        # Vision-estimated distance (meters)
    'angle': 0.12,           # Horizontal angle (radians, +=right camera convention)
    'bbox_clipped': False,   # True if bbox extends beyond image edges
    'class_name': 'bowling-pins',
    'confidence': 0.85,
    'bbox': (100, 120, 450, 480),  # (x1, y1, x2, y2)
}
```

---

### `find_best_target` (nav/target_selector.py)

```python
find_best_target(detections: list[dict]) -> Optional[dict]
```

Returns the detection with minimum `'distance'` value, or `None` if list is empty.

---

## 4. threads/ — Worker Threads

### `ros_thread` (threads/ros_node.py)

Creates `MainGuiNode(Node)` and spins it.

#### `MainGuiNode` — ROS2 Node

| Attribute | Type | Description |
|-----------|------|-------------|
| `navigator` | `Navigator` | Navigation engine instance |
| `is_active` | `bool` | Whether navigation is enabled |
| `last_target` | `dict or None` | Last seen detection |

**Subscriptions**:
- `/map` (OccupancyGrid) — RELIABLE + TRANSIENT_LOCAL QoS
- `/scan` (LaserScan) — BEST_EFFORT QoS
- `/gui/command` (String) — Commands from GUI

**Publishers**:
- `/cmd_vel` (Twist) — Velocity commands

**Timer**: 20Hz → `control_loop()`

#### Control Loop (20Hz)

```
1. Check GO/STOP flags
2. Read latest detections from DetectionStore
3. find_best_target() → closest pin
4. Navigate based on state:
   - Target visible → navigate_to_target()
   - Lost < 3s → drift forward
   - Lost > 3s + close → enter blind approach
   - Lost > 3s + far → search rotate
   - Search timeout → IDLE
   - ARRIVED → stop (terminal)
```

---

### `camera_thread` (threads/camera_worker.py)

```python
camera_thread(shared_state: SharedState) -> None
```

Main camera worker running in its own thread.

#### Detection Priority

1. **DRP-AI** (V2N only): `find_drp_binary()` + `find_drp_model()` → `DrpBinaryDetector`
2. **ONNX CPU** (fallback): `find_model_path()` → `YoloOnnxDetector`

#### Key Functions

| Function | Signature | Returns | Description |
|----------|-----------|---------|-------------|
| `find_model_path` | `()` | `str or None` | Find ONNX model file |
| `detect_objects` | `(frame, detector, estimator)` | `(detections, infer_time)` | Run detection + distance estimation |
| `draw_rect_np` | `(img, x1, y1, x2, y2, color, thickness)` | `None` | Draw rectangle using numpy (avoids DRP-AI conflicts) |

---

## 5. detectors/ — AI Detection Backends

### `Detection` (detectors/base.py)

```python
@dataclass
class Detection:
    class_name: str              # e.g., "bowling-pins"
    class_id: int                # Numeric class ID
    confidence: float            # 0.0-1.0
    bbox: tuple[int,int,int,int] # (x1, y1, x2, y2)
    center: tuple[int,int]       # Auto-calculated (cx, cy)
    center_normalized: tuple[float,float]  # (-1..1, -1..1)
    area: int                    # Auto-calculated
    distance: Optional[float]    # Estimated distance (meters)
```

| Property | Returns | Description |
|----------|---------|-------------|
| `width` | `int` | x2 - x1 |
| `height` | `int` | y2 - y1 |
| `center_x` | `int` | center[0] |
| `center_y` | `int` | center[1] |

| Method | Signature | Description |
|--------|-----------|-------------|
| `normalize_center` | `(frame_width, frame_height)` | Calculate center_normalized: (-1..1) range |

---

### `DetectionResult` (detectors/base.py)

```python
@dataclass
class DetectionResult:
    detections: list[Detection]
    inference_time: float         # Seconds
    timestamp: float              # Unix timestamp
    frame_size: tuple[int,int]    # (width, height)
    success: bool
    error: str
```

| Property/Method | Returns | Description |
|----------------|---------|-------------|
| `has_detections` | `bool` | len(detections) > 0 |
| `best_detection` | `Detection or None` | Highest confidence |
| `filter_by_class(name)` | `list[Detection]` | Filter by class name |
| `filter_by_confidence(min)` | `list[Detection]` | Filter by min confidence |

---

### `DetectorBase` (detectors/base.py) — Abstract Base Class

```python
DetectorBase(confidence_threshold=0.5, target_class="bowling-pins", **kwargs)
```

| Abstract Property | Returns | Description |
|-------------------|---------|-------------|
| `name` | `str` | Detector name |
| `supported_classes` | `list[str]` | Detectable classes |

| Abstract Method | Signature | Description |
|-----------------|-----------|-------------|
| `_load_model` | `()` | Load detection model |
| `_detect_impl` | `(frame: ndarray) -> DetectionResult` | Run inference |

| Public Method | Signature | Returns | Description |
|--------------|-----------|---------|-------------|
| `initialize` | `()` | `bool` | Load model, set initialized flag |
| `shutdown` | `()` | `None` | Release resources |
| `detect` | `(frame: ndarray)` | `DetectionResult` | Run detection with error handling |
| `detect_target` | `(frame: ndarray)` | `Detection or None` | Detect primary target class |
| `get_stats` | `()` | `dict` | Detector statistics |

---

### `YoloOnnxDetector` (detectors/yolo_onnx_detector.py)

```python
YoloOnnxDetector(
    model_path="models/bowling_yolov5.onnx",
    input_size=(640, 640),
    class_names=["bowling-pins"],
    confidence_threshold=0.5,
    nms_threshold=0.45,
    target_class="bowling-pins",
    filter_classes=None,
    **kwargs
)
```

- Auto-detects YOLOv5 vs YOLOv8 from output shape
- ONNX Runtime with ORT_ENABLE_ALL optimization
- 4 intra-op threads for ARM
- NMS (Non-Maximum Suppression) for deduplication

---

### `DrpBinaryDetector` (detectors/drp_binary_detector.py)

```python
DrpBinaryDetector(
    binary_path=None,             # Auto-find at /opt/drp/yolo_detection
    model_dir=None,               # Auto-find at /opt/drp/drpai_model
    confidence_threshold=0.5,
    target_class="bowling-pins",
    class_names=None,
    startup_timeout=30.0,
    **kwargs
)
```

- Spawns C++ subprocess with `--pipe` mode
- Sends frames as binary (width, height, BGR pixels)
- Receives JSON detections on stdout
- 10-20x faster than ONNX CPU on V2N

#### Helper Functions

| Function | Returns | Description |
|----------|---------|-------------|
| `find_drp_binary()` | `str or None` | Locate DRP-AI executable |
| `find_drp_model()` | `str or None` | Locate DRP-AI model directory |

---

## 6. gui/ — GTK3 User Interface

### `MainGUI` (gui/main_window.py)

```python
MainGUI(shared_state: SharedState)  # Extends Gtk.Window
```

| Method | Description |
|--------|-------------|
| `on_draw(widget, cr)` | Render map and camera panels |
| `on_quit(widget)` | Shutdown and exit |

### `SettingsWindow` (gui/settings_window.py)

```python
SettingsWindow(shared_state: SharedState)  # Extends Gtk.Window
```

5 tabs: Navigation, Blind Approach, Detection, Obstacle, Calibration.

### Panel Functions

```python
# gui/panels/map_panel.py
draw_map_panel(cr, x, y, w, h, state)    # Render SLAM map

# gui/panels/camera_panel.py
draw_camera_panel(cr, x, y, w, h, state) # Render camera feed
```

### Display and Theme

```python
# gui/display.py
setup_display()     # Auto-detect Wayland/X11, set environment variables

# gui/theme.py
apply_theme()       # Apply dark CSS theme to all GTK widgets
```

---

## 7. hardware/ — Hardware Abstractions

### Arduino

```python
from bowling_target_nav.hardware import ArduinoBridge, MockArduino, create_arduino
```

#### `ArduinoBridge` (hardware/arduino.py)

```python
ArduinoBridge(
    device_path="/dev/ttyACM0",
    baudrate=115200,
    timeout=0.5,
    auto_reconnect=True
)
```

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `connect` | `()` | `bool` | Open serial, wait for READY |
| `disconnect` | `()` | `None` | Send STOP, close serial |
| `send_command` | `(command: str)` | `str or None` | Send and get response |
| `set_velocity` | `(vx, vy, wz: float)` | `bool` | Send VEL with mecanum IK |
| `move` | `(direction, speed, ticks)` | `bool` | Send timed movement |
| `turn` | `(speed, ticks)` | `bool` | Send rotation |
| `read_encoders` | `()` | `EncoderData or None` | Read 4 encoder values |
| `stop` | `()` | `bool` | Emergency stop |
| `calibrate` | `()` | `bool` | Start motor calibration |

#### `ArduinoBridgeDriver` (hardware/arduino_bridge.py)

Production serial driver with background read/reconnect threads.

```python
ArduinoBridgeDriver(config: Optional[ArduinoConfig] = None, on_response: Optional[Callable] = None, on_state_change: Optional[Callable] = None)
```

#### `ArduinoState` (Enum)

```python
class ArduinoState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"
```

#### `ArduinoConfig` (Dataclass)

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `port` | str | `"/dev/ttyACM0"` | Serial port path |
| `baudrate` | int | `115200` | Serial baud rate |
| `timeout` | float | `0.1` | Serial read timeout |
| `reconnect_interval` | float | `2.0` | Auto-reconnect delay |
| `command_timeout` | float | `0.3` | Max wait for response |
| `max_retries` | int | `3` | Command retry count |

#### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `start` | `()` | `None` | Launch background threads |
| `stop` | `()` | `None` | Stop threads, disconnect |
| `connect` | `()` | `bool` | Establish serial connection, wait for READY |
| `disconnect` | `()` | `None` | Send STOP, close serial |
| `send_command` | `(command: str, *args)` | `bool` | Send plain-text command |
| `send_velocity` | `(vx, vy, wz: int)` | `bool` | Send VEL (PWM values) |
| `send_move` | `(direction, speed, ticks)` | `bool` | Timed movement |
| `send_turn` | `(speed, ticks, clockwise=False)` | `bool` | Send rotation (positive ticks=CCW) |
| `send_stop` | `()` | `bool` | Emergency stop |
| `find_arduino_port` | `()` | `str or None` | Auto-detect by VID/PID |

#### `EncoderData`

```python
@dataclass
class EncoderData:
    values: list[int]   # [FL, RL, RR, FR]
    timestamp: float
    # Properties: front_left, rear_left, rear_right, front_right
```

#### Constants

```python
WHEEL_RADIUS_M = 0.04      # 80mm diameter
WHEELBASE_M = 0.190         # Front-rear distance
TRACK_WIDTH_M = 0.210       # Left-right distance
ENCODER_CPR = 4320          # Counts per revolution
SPEED_MIN = 20              # Minimum PWM
SPEED_MAX = 255             # Maximum PWM
WATCHDOG_TIMEOUT_MS = 200   # VEL mode watchdog
```

---

### Camera

```python
from bowling_target_nav.hardware import CameraCapture, MockCamera, create_camera
```

#### `CameraCapture` (hardware/camera.py)

```python
CameraCapture(device_id=0, width=640, height=480, fps=30, auto_reconnect=True)
```

| Method | Returns | Description |
|--------|---------|-------------|
| `open()` | `bool` | Open camera device |
| `close()` | `None` | Release camera |
| `read()` | `FrameData or None` | Capture single frame |
| `get_frame()` | `ndarray or None` | Convenience: frame only |

#### `FrameData`

```python
@dataclass
class FrameData:
    frame: np.ndarray    # BGR image
    timestamp: float
    frame_number: int
    width: int
    height: int
    # Property: size → (width, height)
```

#### `MockCamera`

```python
MockCamera(pattern="gradient", frame_delay=0.033, static_image=None)
# Patterns: "gradient", "noise", "checkerboard", "moving", "static"
```

---

### LiDAR

```python
from bowling_target_nav.hardware import LidarBridge, MockLidar, create_lidar
```

#### `LidarBridge` (hardware/lidar.py)

```python
LidarBridge(device_path="/dev/ttyUSB0", baudrate=115200, min_range=0.15, max_range=12.0, auto_reconnect=True)
```

| Method | Returns | Description |
|--------|---------|-------------|
| `connect()` | `bool` | Connect and verify health |
| `disconnect()` | `None` | Stop motor, disconnect |
| `start_scanning()` | `bool` | Begin continuous scans |
| `stop_scanning()` | `None` | Stop motor |
| `get_scan()` | `LidarScan or None` | Get next scan |

#### `LidarScan`

```python
@dataclass
class LidarScan:
    points: list[LidarPoint]
    timestamp: float
    scan_number: int
    # Properties: num_points, min_distance, min_distance_angle
    # Methods: get_range(start, end), get_front_distance(range=30), to_numpy(), to_cartesian()
```

#### `LidarPoint`

```python
@dataclass
class LidarPoint:
    angle: float       # Degrees (0-360)
    distance: float    # Meters
    quality: int       # Signal quality (0-255)
    # Properties: angle_rad, x, y
    # Method: as_tuple() → (x, y)
```

---

### Factory Functions

```python
create_arduino(use_mock=False, config=None, auto_connect=True, **kwargs) -> ArduinoBase
create_camera(use_mock=False, config=None, auto_open=True, **kwargs) -> CameraBase
create_lidar(use_mock=False, config=None, auto_connect=True, **kwargs) -> LidarBase
```

---

## 8. utils/ — Utilities

### `DistanceEstimator` (utils/distance_estimator.py)

```python
DistanceEstimator(
    reference_box_height=100.0,   # px at reference distance
    reference_distance=1.0,       # meters
    frame_width=320,
    frame_height=240,
    horizontal_fov=60.0           # degrees
)
```

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `estimate` | `(detection)` | `(distance, angle)` | Distance (m) and angle (rad) |
| `estimate_position` | `(detection)` | `(x, y)` | Position in robot frame |
| `get_relative_size` | `(detection)` | `float` | Size ratio vs reference |
| `get_normalized_position` | `(detection)` | `(nx, ny)` | Normalized (-1..1) position |
| `is_centered` | `(detection, threshold=0.1)` | `bool` | Near frame center? |
| `calibrate` | `(detection, known_distance)` | `None` | Update reference from known |

**Distance formula**: `distance = ref_distance * (ref_box_height / current_height)`

**Angle formula**: `angle = atan2(offset_x, focal_length)`

---

## 9. nodes/ — ROS2 Entry Points

All entry points are registered in `setup.py` as console scripts.

| Entry Point | Command | Description |
|-------------|---------|-------------|
| `main_gui` | `ros2 run bowling_target_nav main_gui` | Primary: 3-thread GUI application |
| `vision_node` | `ros2 run bowling_target_nav vision_node` | Camera + YOLO detection publisher |
| `arduino_driver_node` | `ros2 run bowling_target_nav arduino_driver_node` | Motor control serial driver |
| `target_follower_node` | `ros2 run bowling_target_nav target_follower_node` | Navigation toward detected targets |
| `target_gui_node` | `ros2 run bowling_target_nav target_gui_node` | Alternative detection GUI |
| `odometry_node` | `ros2 run bowling_target_nav odometry_node` | Wheel odometry + TF broadcaster |
| `map_viewer_node` | `ros2 run bowling_target_nav map_viewer_node` | SLAM map viewer |
| `slam_camera_gui` | `ros2 run bowling_target_nav slam_camera_gui` | Combined SLAM + camera GUI |
| `autonomous_explorer` | `ros2 run bowling_target_nav autonomous_explorer` | Autonomous exploration + target pushing |

---

## 10. Configuration Files

### config/robot_config.yaml

Main robot configuration with sections: detection, camera, lidar, arduino, navigation, distance_estimation, gui, logging.

### config/nav2_params.yaml

Nav2 stack parameters: AMCL localization (OmniMotionModel), DWB controller (mecanum-aware), local/global costmaps, collision monitor, velocity smoother.

### config/target_nav_params.yaml

Vision node and target follower node parameters: detector settings, camera config, distance calibration, approach behavior.

### config/cartographer.lua

Cartographer SLAM: 2D trajectory builder, scan matching, map resolution (5cm), no loop closure (CPU efficiency).

### Launch Files

| File | Purpose |
|------|---------|
| `bringup.launch.py` | Hardware drivers (LiDAR, Arduino, odometry) |
| `mapping.launch.py` | Cartographer SLAM |
| `navigation.launch.py` | Full Nav2 stack |
| `bowling_target_nav.launch.py` | Vision + navigation nodes |
| `autonomous.launch.py` | Full autonomous system |
| `slam.launch.py` | SLAM mapping only |
| `record.launch.py` | Rosbag recording |
