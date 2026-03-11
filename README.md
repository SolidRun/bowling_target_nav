# Bowling Target Navigation — RZ/V2N Robot

Autonomous bottle detection and navigation for the Renesas RZ/V2N mecanum robot. DRP-AI hardware-accelerated YOLO detection, Cartographer SLAM, holonomic VFH navigation, all in a fullscreen GTK3 GUI.

## Architecture

3 processes on 3 CPU cores:

| Core | Process | Modules | Rate |
|------|---------|---------|------|
| 0 | GUI (main) | GTK3 rendering, settings, user input | 30fps |
| 1 | ROS2 + Navigation | /map, /scan, TF2, control loop, Navigator | 20Hz |
| 2 | Camera + DRP-AI | C++ stream inference, distance estimation | ~15fps |

IPC via SharedMemory (frames/map/laser), mp.Array (pose/nav state), mp.Queue (commands).

Set `BOWLING_NAV_MULTIPROCESS=0` for single-process threading mode.

## Detection Pipeline

```
DRP-AI C++ Binary (stream mode)
    │
    ├── Camera frames → /dev/shm/v2n_camera (zero-copy)
    └── JSON detections → stdout
            │
            ▼
    camera_worker.py filters:
    1. Confidence threshold (default 0.30)
    2. Min box size (10x15px — reject noise)
    3. Max box size (80% frame — reject false positives)
    4. Aspect ratio (bottle taller than wide)
    5. Bbox size tracker (rolling median, reject outliers)
            │
            ▼
    Distance estimator (bbox height → meters)
            │
            ▼
    IPC → ROS process → Navigator
```

## Navigation Engine

```
navigate_to_target(target)
    │
    ├── Distance fusion: camera (primary) + LiDAR (fallback when bbox clipped)
    ├── Sensor offset compensation (camera 14cm, LiDAR 5cm from center)
    │
    ├── Far (>0.40m): Full VFH navigation with obstacle avoidance
    │   └── Speed scales with distance, mecanum strafing through gaps
    │
    ├── Close (<0.40m): Direct approach at min_speed (0.07 m/s)
    │   └── Bypasses VFH — bottle IS the obstacle
    │
    ├── Arrival (≤0.10m): Multi-signal check (vision + LiDAR + bbox_clipped)
    │   └── 0.3s temporal confirmation
    │
    └── Target lost:
        ├── <3s: Drift forward at min_speed
        ├── Close + lost: Blind approach (dead-reckon via TF2 map-frame)
        └── >3s: Search (360° scan → Archimedean spiral)
```

## Settings (GUI)

Only 2 detection sliders (everything else has safe hardcoded defaults):

| Setting | Range | Default | Description |
|---------|-------|---------|-------------|
| Confidence | 0.05–0.95 | 0.30 | Lower = more detections, more false positives |
| Detection Memory | 0.3–5.0s | 1.0s | How long to keep a lost detection |

Navigation params tunable in Nav/Search/Blind/Obstacle tabs. Sensor mounting in Setup tab. All auto-saved to `~/.config/bowling_target_nav/calibration.json`.

## Key Files

| File | Purpose |
|------|---------|
| `nodes/main_gui.py` | Entry point — spawns 3 processes or 3 threads |
| `threads/ros_node.py` | ROS2 node + 20Hz control loop |
| `threads/camera_worker.py` | DRP-AI detection + filtering |
| `nav/navigator.py` | Holonomic nav, VFH, blind approach, spiral search |
| `nav/target_selector.py` | Pick closest valid detection |
| `state/detection_store.py` | All tunable params + calibration persistence |
| `gui/settings_window.py` | 8-tab settings UI |
| `gui/panels/map_panel.py` | SLAM map + diagnostics rendering |
| `gui/panels/camera_panel.py` | Camera feed + detection overlay |
| `ipc/hub.py` | SharedMemory IPC hub |
| `processes/ros_process.py` | ROS process with inbound/outbound sync |
| `processes/camera_process.py` | Camera process with settings watcher |
| `detectors/drp_binary_detector.py` | DRP-AI stream + pipe backends |

## Deploy to V2N

```bash
# From PC (use /* glob, NOT trailing /)
scp -r src/bowling_target_nav/bowling_target_nav/* \
  root@192.168.50.1:/root/ros2_ws/src/bowling_target_nav/bowling_target_nav/

# Clear Python cache on V2N
ssh root@192.168.50.1 "find /root/ros2_ws -name '__pycache__' -exec rm -rf {} + 2>/dev/null"

# Restart GUI
ssh root@192.168.50.1 "pkill -f main_gui; pkill -f bringup"
# Then press the floating launcher button on screen, or:
ssh root@192.168.50.1 "cd /root/ros2_ws/src/bowling_target_nav/scripts && ./bowling_gui.sh &"
```

## TF Tree

```
map ──► odom ──► base_link ──► laser
  │       │           ├──► camera_link
  │       │           └──► wheels
  │       └── odometry_node (encoder ticks)
  └── cartographer (SLAM)
```

**Critical**: Cartographer must use `provide_odom_frame=false` + `published_frame="odom"` since odometry_node already publishes `odom→base_link`. Constraint builder disabled on V2N aarch64 to prevent mutex deadlock.

## ROS2 Topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/cmd_vel` | Twist | Publish | Motor velocity |
| `/scan` | LaserScan | Subscribe | LiDAR points |
| `/map` | OccupancyGrid | Subscribe | SLAM grid |
| `/odom` | Odometry | Subscribe (via TF) | Robot pose |
| `/gui/command` | String | Subscribe | Remote GO/STOP |
| `/arduino/cmd` | String | Publish | Motor calibration |

## License

MIT
