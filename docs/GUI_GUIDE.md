# GUI User Guide

> Complete guide to the V2N Robot Control GUI — layout, controls, settings, and customization.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Main Window Layout](#2-main-window-layout)
3. [Map Panel (Left)](#3-map-panel-left)
4. [Camera Panel (Right)](#4-camera-panel-right)
5. [Control Bar](#5-control-bar)
6. [Status Bar](#6-status-bar)
7. [Keyboard Shortcuts](#7-keyboard-shortcuts)
8. [Settings Window](#8-settings-window)
9. [Display Configuration](#9-display-configuration)
10. [Theme and Styling](#10-theme-and-styling)

---

## 1. Overview

The GUI is a fullscreen GTK3 application running on Wayland/Weston. It displays real-time SLAM mapping, camera feed with AI detection overlays, and navigation controls — all in a single window.

```
┌─────────────────────────────────────────────────────────────────┐
│  V2N Robot Control          SLAM + Camera + Navigation          │
│  ┌────────────────────────┐  ┌────────────────────────────────┐ │
│  │                        │  │                    ┌──────────┐│ │
│  │      SLAM Map          │  │     Camera Feed    │NAVIGATING││ │
│  │                        │  │                    └──────────┘│ │
│  │   ● Robot (green)      │  │   ┌────┐                      │ │
│  │   · Laser (red)        │  │   │ pin│ 0.45m  12°           │ │
│  │   ◇ Target (magenta)   │  │   └────┘                      │ │
│  │   ─ Nav path (magenta) │  │                                │ │
│  │                        │  │              v=0.12 m/s  w=5°/s│ │
│  │  Robot: (1.2, 0.5) 45° │  │  Detection: Pins: 0.45m, 12° │ │
│  └────────────────────────┘  └────────────────────────────────┘ │
│                                                                  │
│  [GO TO TARGET]  [STOP]  ● NAVIGATING 0.45m      [SETTINGS][QUIT]│
└─────────────────────────────────────────────────────────────────┘
```

**Technology**: GTK3 with Cairo 2D graphics, GdkPixbuf for image display, Pango for text rendering.

**Refresh Rate**: 30 FPS (33ms timer via `GLib.timeout_add`).

---

## 2. Main Window Layout

The window is divided into three vertical sections:

### Title Bar (Top 50px)
- **"V2N Robot Control"** — Blue (#58a6ff), 26px font
- **"SLAM + Camera + Navigation"** — Gray (#8b949e), 14px subtitle

### Content Area (Middle, fills remaining space)
- Split 50/50 into left (Map) and right (Camera) panels
- 10px margin around edges, 10px gap between panels
- Each panel has a dark background (#161b22) with gray border (#30363d)

### Control Bar (Bottom)
- Horizontal box with 20px spacing, 10px margins
- Buttons: GO, STOP, (expandable status label), SETTINGS, QUIT

---

## 3. Map Panel (Left)

**Header**: "SLAM Map" in blue (0.345, 0.651, 1.0), 15px font

### What's Displayed

| Element | Color | Description |
|---------|-------|-------------|
| Free space | Dark gray (50,50,50) | Navigable areas |
| Occupied | Orange (0,200,255 BGR) | Walls and obstacles |
| Unknown | Very dark (30,30,30) | Unexplored areas |
| Robot | Green circle + arrow | Current position and heading |
| Laser points | Red dots (2px) | LiDAR scan in real-time |
| Nav target | Magenta diamond | Target position in map frame |
| Nav path | Magenta line | Line from robot to target |
| Grid | Gray lines | Drawn when zoom > 15px/meter |

### How Map Rendering Works

1. The `/map` topic (OccupancyGrid) is received by the ROS thread
2. Each cell value is mapped to a color: -1→unknown, 0→free, >0→occupied
3. The image is flipped vertically (ROS origin is bottom-left, display is top-left)
4. Scaled to fit the panel while preserving aspect ratio
5. Robot position is obtained from TF (`map → base_link`)
6. LiDAR points are transformed from robot frame to map frame and plotted
7. Navigation target (if any) is drawn as a diamond with a line from robot

### Info Label (Bottom)
```
Robot: (1.23, 0.45) 67°
```
Shows the robot's position in the map frame and heading in degrees.

---

## 4. Camera Panel (Right)

**Header**: "Camera + Detection" in orange (0.902, 0.557, 0.149), 15px font

### What's Displayed

| Element | Appearance | Drawn By | Description |
|---------|-----------|----------|-------------|
| Camera frame | RGB video | camera_panel (Cairo) | Live camera feed, aspect-ratio preserved |
| Detection boxes | Green rectangles | camera_worker (OpenCV) | Bounding boxes burned into frame buffer |
| Crosshair | Green circle + ticks | camera_panel (Cairo) | Centered on closest detection |
| Distance label | Green text, 13px | camera_panel (Cairo) | `"0.45m  12°"` near crosshair |
| CLIPPED warning | Red text | camera_panel (Cairo) | Shown when bbox extends beyond frame edge |
| State badge | Colored rectangle | camera_panel (Cairo) | Top-right corner: NAVIGATING, SEARCHING, etc. |
| Speed indicator | Cyan text, 12px | camera_panel (Cairo) | `"v=0.12 m/s  ω=5°/s"` at bottom-left |

### Detection Overlay Colors

Bounding boxes are drawn by the camera thread (OpenCV on the frame buffer), not by the Cairo camera panel. The camera panel only draws the crosshair on the closest detection.

| Condition | Color (BGR) | Display Color | Meaning |
|-----------|-------------|---------------|---------|
| Fresh detection (< 0.3s) | (0, 255, 0) | Green | Just detected |
| Stale detection (> 0.3s) | (0, 200, 200) | Yellowish-green | Using cached result |

### State Badge Colors (Camera Panel)

These are Cairo RGB values from `camera_panel.py`, rendered as semi-transparent rectangles in the top-right corner of the camera feed:

| State | RGB | Hex | Display |
|-------|-----|-----|---------|
| NAVIGATING | (0.137, 0.533, 0.212) | #238836 | Dark green |
| SEARCHING | (0.886, 0.686, 0.0) | #E2AF00 | Gold |
| BLIND APPROACH | (0.902, 0.557, 0.149) | #E68E26 | Orange |
| ARRIVED | (0.122, 0.435, 0.918) | #1F6FEA | Blue |
| IDLE | (0.35, 0.38, 0.42) | #59616B | Gray |
| ERROR | (0.855, 0.212, 0.200) | #DA3633 | Red |

> Note: The badge displays `nav_state.replace("_", " ")`, so "BLIND_APPROACH" appears as "BLIND APPROACH".

### Info Label (Bottom)
```
Detection: Pins: 0.45m, 12.3°
```

---

## 5. Control Bar

| Button | Size | Style | Keyboard | Action |
|--------|------|-------|----------|--------|
| **GO TO TARGET** | 200x60px | Green (suggested-action) | `G` | Start navigating to closest pin |
| **STOP** | 200x60px | Red (destructive-action) | `Space` or `S` | Emergency stop |
| **SETTINGS** | 120x60px | Blue (settings-btn) | — | Open settings window |
| **QUIT** | 100x60px | Orange (quit-btn) | `Q` or `ESC` | Quit application |

### Button Behavior

- **GO**: Sets `state.nav.request_go()` flag. The ROS thread picks it up on next control loop cycle (within 50ms). Robot starts searching/navigating.
- **STOP**: Sets `state.nav.request_stop()` flag. Robot stops immediately. Navigation state resets to IDLE.
- **SETTINGS**: Opens the Settings window (non-modal, can coexist with main window).
- **QUIT**: Calls `state.request_shutdown()`, stops GTK main loop, all threads clean up within 5 seconds.

---

## 6. Status Bar

Located between STOP and SETTINGS buttons, the status label shows navigation state with color-coded Pango markup.

### Status Format

```
● NAVIGATING  0.45m  v=0.12m/s  ⚠ obstacle 0.3m
```

### State Indicators

| State | Indicator | Color | Additional Info |
|-------|-----------|-------|-----------------|
| IDLE | `◯ IDLE` | Gray (#8b949e) | — |
| SEARCHING | `⬤ SEARCHING` | Gold (#e3b341) | Search time in seconds |
| NAVIGATING | `⬤ NAVIGATING` | Green (#3fb950) | Distance to target |
| BLIND_APPROACH | `⬤ BLIND_APPROACH` | Orange (#ffa657) | Distance remaining |
| ARRIVED | `⬤ ARRIVED` | Blue (#58a6ff) | — |
| ERROR | `⬤ ERROR` | Red (#f85149) | Error message |

### Optional Status Additions

- **Obstacle warning**: `⚠ obstacle 0.30m` (when obstacle < 0.3m)
- **Speed**: `v=0.12m/s` (when speed > 0.01 m/s)
- **Lost time**: `lost 2.1s` (when target not visible)

---

## 7. Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `G` | GO — Start navigation |
| `Space` | STOP — Emergency stop |
| `S` | STOP — Emergency stop (alternative) |
| `Q` | Quit application |
| `ESC` | Quit application |

All keyboard events are captured by the main window's `key-press-event` handler.

---

## 8. Settings Window

A 540x620px dialog window with 5 tabs for real-time parameter tuning. All changes take effect **immediately** — no save/apply button needed.

### Tab 1: Navigation

Controls the main navigation behavior.

| Parameter | Range | Default | Unit | Effect |
|-----------|-------|---------|------|--------|
| Linear Speed | 0.05–0.30 | 0.15 | m/s | Forward speed toward target |
| Min Speed | 0.05–0.20 | 0.10 | m/s | Motor dead zone threshold |
| Angular Speed | 0.1–1.0 | 0.5 | rad/s | Maximum rotation speed |
| Approach Distance | 0.05–0.50 | 0.15 | m | Distance to declare "arrived" |
| Lost Timeout | 1.0–10.0 | 3.0 | s | Seconds before switching to blind/search |
| Search Timeout | 10.0–60.0 | 30.0 | s | Safety timeout (search also stops after 360° scan) |
| Search Angular Speed | 0.1–1.0 | 0.4 | rad/s | Rotation speed when searching |

### Tab 2: Blind Approach

Controls dead-reckoning behavior when camera loses sight of pin.

| Parameter | Range | Default | Unit | Effect |
|-----------|-------|---------|------|--------|
| Entry Distance | 0.30–1.50 | 0.80 | m | Switch to blind approach below this distance |
| Approach Speed | 0.05–0.20 | 0.10 | m/s | Slow forward speed during blind approach |
| Timeout | 3.0–15.0 | 8.0 | s | Abort blind approach after this |
| LiDAR Stop | 0.05–0.30 | 0.12 | m | Stop if LiDAR detects something this close |
| Arrival Margin | 0.05–0.25 | 0.10 | m | Declare arrived within this distance |

### Tab 3: Detection

Controls AI detection sensitivity and timing.

| Parameter | Range | Default | Unit | Effect |
|-----------|-------|---------|------|--------|
| Confidence Threshold | 0.10–0.90 | 0.35 | — | Minimum YOLO confidence to accept detection |
| Detection Interval | 0.5–5.0 | 2.0 | s | Minimum time between YOLO inference runs (DRP-AI) |
| Detection Expiry | 0.5–5.0 | 1.5 | s | Discard detections older than this |

**Tips**:
- Lower confidence threshold → more detections but more false positives
- Higher detection interval → less CPU load but slower reaction
- Higher expiry → targets "remembered" longer, but may navigate to stale positions

### Tab 4: Obstacle

Controls LiDAR-based obstacle avoidance.

| Parameter | Range | Default | Unit | Effect |
|-----------|-------|---------|------|--------|
| Stop Distance | 0.10–0.50 | 0.25 | m | Emergency stop when obstacle this close |
| Slowdown Distance | 0.20–1.00 | 0.50 | m | Start slowing down at this distance |
| Robot Half Width | 0.05–0.30 | 0.15 | m | Width of front corridor for obstacle check |

### Tab 5: Calibration

Tools for calibrating distance estimation and testing motors.

#### Distance Calibration

1. Place a bowling pin at a **known distance** from the camera
2. Enter the distance in the spin button (0.3–3.0 m)
3. Press **CALIBRATE**
4. The system captures the current bbox height and stores it as reference
5. Result shows: `"Calibrated: 142px at 1.0m"`

#### Motor Test

Test each movement direction for 1 second:

| Button | Motor Command | Duration |
|--------|--------------|----------|
| FORWARD | linear.x = +speed | 1s |
| BACKWARD | linear.x = -speed | 1s |
| LEFT | linear.y = +speed | 1s |
| RIGHT | linear.y = -speed | 1s |
| TURN L | angular.z = +0.5 | 1s |
| TURN R | angular.z = -0.5 | 1s |

Speed is configurable via the speed spin button (0.05–0.30 m/s).

#### Reset Odometry

Publishes an `Empty` message to `/reset_odom` topic, resetting wheel encoder integration to (0, 0, 0).

---

## 9. Display Configuration

### Automatic Display Detection

The GUI auto-detects the display server on startup (`gui/display.py`):

1. **Wayland (V2N default)**: Detects Weston process, finds Wayland socket
2. **X11 (PC fallback)**: Uses `DISPLAY` environment variable
3. **Fallback**: Defaults to Wayland with `/run` as runtime directory

### Environment Variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `GDK_BACKEND` | `wayland` | GTK display backend |
| `WAYLAND_DISPLAY` | `wayland-0` | Wayland socket name |
| `XDG_RUNTIME_DIR` | `/run` | Runtime directory for socket |
| `OPENCV_OPENCL_DEVICE` | `disabled` | Prevent DRP-AI conflicts |

### Troubleshooting Display Issues

```bash
# Check Wayland
pgrep -x weston
ls /run/wayland-0

# Check X11
echo $DISPLAY

# Force X11 mode
export GDK_BACKEND=x11
export DISPLAY=:0
```

---

## 10. Theme and Styling

The GUI uses a comprehensive dark theme applied via CSS (`gui/theme.py`).

### Color Palette

| Element | Hex | RGB | Usage |
|---------|-----|-----|-------|
| Background | #0d1117 | (13, 17, 23) | Window background |
| Panel | #161b22 | (22, 27, 34) | Panel backgrounds |
| Border | #30363d | (48, 54, 61) | Panel borders |
| Text primary | #e6edf3 | (230, 237, 243) | Main text |
| Text secondary | #c9d1d9 | (201, 209, 217) | Labels |
| Green (GO) | #238636 | (35, 134, 54) | Suggested action |
| Red (STOP) | #da3633 | (218, 54, 51) | Destructive action |
| Blue (settings) | #1f6feb | (31, 111, 235) | Settings button |
| Orange (quit) | #6e4000 | (110, 64, 0) | Quit button |
| Slider trough | #21262d | (33, 38, 45) | Slider background |
| Slider highlight | #1f6feb | (31, 111, 235) | Active slider fill |

### Button Styles

All buttons: 16px bold font, 6px border-radius, `background-image: none` (disables Adwaita gradients for Weston compatibility).

### Widget Hierarchy

```
MainGUI (Gtk.Window, fullscreen)
└── VBox
    ├── DrawingArea (map + camera rendering)
    └── HBox (control bar)
        ├── Button "GO TO TARGET" (suggested-action)
        ├── Button "STOP" (destructive-action)
        ├── Label (status, Pango markup, expandable)
        ├── Button "SETTINGS" (settings-btn)
        └── Button "QUIT" (quit-btn)

SettingsWindow (Gtk.Window, 540x620)
└── Notebook (5 tabs)
    ├── Tab "Navigation" → VBox of sliders
    ├── Tab "Blind Approach" → VBox of sliders
    ├── Tab "Detection" → VBox of sliders
    ├── Tab "Obstacle" → VBox of sliders
    └── Tab "Calibration"
        ├── Distance Calibration section
        ├── Motor Test section (2x3 button grid)
        └── Odometry Reset section
```
