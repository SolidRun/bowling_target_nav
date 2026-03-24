# Target Navigation — RZ/V2N Robot

Autonomous bowling-pin detection and navigation for the Renesas RZ/V2N mecanum robot. DRP-AI hardware-accelerated YOLO detection, holonomic VFH navigation with odometry-only localization, all in a fullscreen GTK3 GUI.

## Architecture (3-process, lock-free struct SHM)

3 Python processes on 4 CPU cores with lock-free SPSC struct SHM IPC:

| Core | Process | Role | Rate |
|------|---------|------|------|
| 0 | GUI (main) | Pure GTK -- reads struct SHM directly, no ROS2 | 10 FPS |
| 1 | Nav (nice -5) | ROS2 navigation, Kalman tracker, motor commands | 20 Hz |
| 2 | Camera | DRP-AI subprocess management, detection tracking | ~10 FPS |
| 3 | DRP-AI C++ binary | YOLO inference (pinned to isolate from nav) | ~10 FPS |

### Data flow (all struct SHM, no JSON for real-time data)

```
Camera (Core 2) --DetShmWriter--> Nav (Core 1) via DetShmReader [20Hz poll]
Camera (Core 2) --DetShmWriter--> GUI (Core 0) via DetShmReader [20Hz poll]
Camera (Core 2) --/dev/shm/v2n_camera--> GUI (Core 0) [raw BGR frames from C++]
Nav (Core 1) --NavShmWriter--> GUI (Core 0) via NavShmReader [20Hz poll]
Nav (Core 1) --LaserShmWriter--> GUI (Core 0) via LaserShmReader [20Hz poll]
GUI (Core 0) --CmdRingBuffer.push()--> Nav (Core 1) via CmdRingBuffer.pop() [20Hz poll]
```

### Lock-free protocol

- SPSC (Single Producer, Single Consumer) per channel
- Torn-read protection via sequence number sandwich (seq_start == seq_end)
- No locks, no JSON serialization, no Bridge process
- struct.pack_into / struct.unpack_from for zero-copy binary I/O

## Detection Pipeline

Target: bowling-pin (0.28m height x 0.08m width).

```
DRP-AI C++ Binary (Core 3, stream mode)
    │
    ├── Camera frames → /dev/shm/v2n_camera (zero-copy, GUI reads directly)
    └── Detections → /dev/shm/v2n_detections (binary struct)
            │
            ▼
    camera_worker.py (Core 2) filters:
    1. C++ applies: confidence threshold, NMS, size/aspect filters
    2. Python applies: temporal tracking (DetectionTracker)
    3. Shape validation (width/height ratio vs bowling-pin dimensions)
    4. Python pinhole model distance estimation (not C++)
            │
            ▼
    DetShmWriter → Nav (Core 1) via DetShmReader [20Hz poll]
    DetShmWriter → GUI (Core 0) via DetShmReader [20Hz poll]
```

## Navigation Engine

```
navigate_to_target(target)
    │
    ├── Distance fusion: camera (primary) + LiDAR (fallback when bbox clipped)
    ├── Sensor offset compensation (camera 15cm, LiDAR 12cm from center)
    │
    ├── Far (>0.45m): Full VFH navigation with obstacle avoidance
    │   └── Speed scales with distance, mecanum strafing through gaps
    │
    ├── Close (<0.45m): Direct approach at min_speed (0.10 m/s)
    │   └── Bypasses VFH — target IS the obstacle
    │
    ├── Arrival (≤0.22m): Multi-signal check (vision + LiDAR + bbox_clipped)
    │   └── 0.3s temporal confirmation
    │
    └── Target lost:
        ├── <3s: Drift forward at min_speed
        ├── Close + lost: Blind approach (dead-reckon via TF2 map-frame)
        └── >3s: Search (360° scan → Archimedean spiral)
```

## Settings (GUI)

5 main tabs with sub-tabs, covering all tunable parameters:

| Tab | Sub-tabs | Key Parameters |
|-----|----------|----------------|
| Navigate | Speed, Target, Approach | Linear/angular speed, approach distance, lost timeout |
| Search | Scan, Spiral | 360° scan speed, spiral radius/growth/timeout |
| Sensors | Detection, Calibration, Mounting, DRP-AI | Confidence (0.05–0.95, default 0.30), detection memory (0.3–5.0s, default 4.0s), DRP-AI frequencies |
| Radar | (flat) | Radar range, point size, grid toggle, nav target overlay |
| Tools | Drive, Motors, System | Hold-to-move motor test, Arduino calibration, reset defaults |

All auto-saved to `~/.config/target_nav/calibration.json` with 2s debounce.

## Key Files

| File | Purpose |
|------|---------|
| `app/main.py` | Entry point — spawns 3 processes, SHM poll thread |
| `app/nav_node.py` | ROS2 node + 20Hz control loop (Core 1) |
| `app/camera_node.py` | Camera ROS2 node — manages DRP-AI subprocess (Core 2) |
| `app/camera_worker.py` | DRP-AI detection + filtering pipeline |
| `app/nav_controller.py` | Navigation state machine (IDLE → NAVIGATING → ...) |
| `app/target_tracker.py` | Kalman-filtered target tracker with LiDAR fusion |
| `ipc/shm_struct.py` | Lock-free SPSC struct SHM readers/writers |
| `nav/navigator.py` | Holonomic nav, VFH, blind approach, spiral search |
| `config.py` | All tunable params (defaults + validation ranges) |
| `state/settings_store.py` | Runtime settings + calibration persistence |
| `gui/settings_window.py` | 5-tab settings UI (with sub-tabs) |
| `gui/panels/map_panel.py` | Radar view + diagnostics rendering |
| `gui/panels/camera_panel.py` | Camera feed + nav-state badge + info bar |
| `detectors/drp_binary_detector.py` | DRP-AI stream + pipe backends |

## Deploy to V2N

```bash
# From PC (use /* glob, NOT trailing /)
scp -r src/target_nav/target_nav/* \
  root@192.168.50.1:/root/ros2_ws/src/target_nav/target_nav/

# Clear Python cache on V2N
ssh root@192.168.50.1 "find /root/ros2_ws -name '__pycache__' -exec rm -rf {} + 2>/dev/null"

# Restart GUI
ssh root@192.168.50.1 "pkill -f main_gui; pkill -f bringup"
# Then press the floating launcher button on screen, or:
ssh root@192.168.50.1 "cd /root/ros2_ws/src/target_nav/scripts && ./gui.sh &"
```

## TF Tree

```
odom ──► base_link ──► laser
  │           ├──► camera_link
  │           └──► wheels
  └── odometry_node (encoder ticks)
```

Localization is odometry-only (no SLAM). The `odom` frame is the fixed reference frame.

## ROS2 Topics

| Topic | Type | Node | Direction | Purpose |
|-------|------|------|-----------|---------|
| `/cmd_vel` | Twist | nav_node | Publish | Motor velocity |
| `/scan` | LaserScan | nav_node | Subscribe | LiDAR points |
| `/arduino/cmd` | String | nav_node | Publish | Raw Arduino commands |
| `/reset_odom` | Empty | nav_node | Publish | Odometry reset trigger |
| `/nav_state` | String | nav_node | Publish | Nav state snapshot (backup, JSON) |
| `/settings_changed` | String | nav_node, camera_node | Subscribe | Reload calibration from disk |
| `/detections` | String | camera_node | Publish | Detection list (backup, JSON) |
| `/detector_mode` | String | camera_node | Publish | Current detector mode |
| `/drpai_restart` | String | camera_node | Subscribe | Restart DRP-AI subprocess |

Note: Primary data flow uses struct SHM, not ROS2 topics. GUI has no ROS2.

## License

MIT
