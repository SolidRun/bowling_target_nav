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

The GUI is a fullscreen GTK3 application running on Wayland/Weston. It displays a real-time radar view (LiDAR + odometry), camera feed with AI detection overlays, and navigation controls — all in a single window.

```
┌─────────────────────────────────────────────────────────────────┐
│  V2N Robot Control          Radar + Camera + Navigation          │
│  ┌────────────────────────┐  ┌────────────────────────────────┐ │
│  │                        │  │                    ┌──────────┐│ │
│  │      Radar View        │  │     Camera Feed    │NAVIGATING││ │
│  │                        │  │                    └──────────┘│ │
│  │   ● Robot (green)      │  │   ┌────┐                      │ │
│  │   · Laser (red)        │  │   │ btl│ 0.45m  12°           │ │
│  │   ◇ Target (magenta)   │  │   └────┘                      │ │
│  │   ─ Nav path (magenta) │  │                                │ │
│  │                        │  │              v=0.12 m/s  w=5°/s│ │
│  │  Robot: (1.2, 0.5) 45° │  │  Detection: Target: 0.45m, 12° │ │
│  └────────────────────────┘  └────────────────────────────────┘ │
│                                                                  │
│  [GO TO TARGET]  [STOP]  ● NAVIGATING 0.45m      [SETTINGS][QUIT]│
└─────────────────────────────────────────────────────────────────┘
```

**Technology**: GTK3 with Cairo 2D graphics, Pango for text rendering.

**Refresh Rate**: 10 FPS (100ms timer via `GLib.timeout_add`) — saves CPU on embedded.

---

## 2. Main Window Layout

The window is divided into three vertical sections:

### Title Bar (Top 50px)
- **"V2N Target Control"** — Blue, 20px font
- **Subtitle** — Shows detector mode (green when DRP-AI active, gray otherwise), 14px

### Content Area (Middle, fills remaining space)
- Split 50/50 into left (Map) and right (Camera) panels
- 10px margin around edges, 10px gap between panels
- Each panel has a dark background (#161b22) with gray border (#30363d)

### Control Bar (Bottom, 50px min height)
- Horizontal box with 20px spacing, 20px margins
- Buttons: GO, STOP, CAP, (expandable status label), QUIT, SETTINGS

---

## 3. Radar Panel (Left)

**Header**: Rendered inline by `draw_map_panel()` — no separate header widget.

### What's Displayed

| Element | Color | Description |
|---------|-------|-------------|
| Robot | Green circle + white arrow | Current position and heading |
| Laser dots | Red dots | Live LiDAR scan points |
| Nav target | Magenta diamond | Target position in odom frame |
| Nav path | Magenta line | Line from robot to target |
| Grid | Gray lines | Drawn when zoom > 15px/meter |

### How Radar Rendering Works

1. Robot position is obtained from TF (`odom → base_link`)
2. LiDAR points from `/scan` are transformed from robot frame to odom frame and plotted
3. Navigation target (if any) is drawn as a diamond with a line from robot

### Diagnostic Overlay (Bottom of Radar Panel)

Diagnostic information is displayed below the radar:

1. **Robot pose**: `Robot: (1.23, 0.45) 67°`
2. **Topic rates + TF age**: `Scan: 5.5Hz (360pts) | TF age: 0.1s` -- Color-coded: green (<0.5s), yellow (<2s), red (>2s)

### Radar Zoom

The radar range can be adjusted via +/- buttons in the top-right corner of the radar panel, or keyboard shortcuts (+/-). Steps: 1, 2, 3, 5, 8, 10, 15, 20 meters.

---

## 4. Camera Panel (Right)

**Header**: Rendered inline by `draw_camera_panel()` — no separate header widget.

### What's Displayed

| Element | Drawn By | Description |
|---------|----------|-------------|
| Camera frame | camera_panel (Cairo) | Live feed from C++ DRP-AI SHM, aspect-ratio preserved |
| Detection boxes | C++ DRP-AI binary | Bounding boxes pre-rendered on the frame in SHM |
| Nav-state badge | camera_panel (Cairo) | Top-right corner: NAVIGATING, SEARCHING, etc. |
| Info bar (80px) | camera_panel (Cairo) | 3-column: camera distance, LiDAR distance, confidence + speed + mode |

Note: The C++ DRP-AI binary draws all detection overlays (bounding boxes, crosshairs, distance labels) on the frame before writing it to shared memory. The Python camera panel just displays the pre-rendered frame plus the nav-state badge and info bar.

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

### Info Bar (Bottom, 80px)

Three-column display:
- **Col 1**: Camera distance (m) + vision angle
- **Col 2**: LiDAR distance (m)
- **Col 3**: Confidence % (color-coded) + speed + detector mode

---

## 5. Control Bar

| Button | Size | Style | Keyboard | Action |
|--------|------|-------|----------|--------|
| **GO TO TARGET** | 180x45px | Green (suggested-action) | `G` | Start navigating to closest target |
| **STOP** | 180x45px | Red (destructive-action) | `Space` or `S` | Emergency stop |
| **CAP** | 60x45px | Default | — | Capture raw camera frame to `/root/captures/` |
| **SETTINGS** | 120x45px | Blue (settings-btn) | — | Switch to settings view (Gtk.Stack) |
| **QUIT** | 100x45px | Orange (quit-btn) | `Q` or `ESC` | Quit application |

### Button Behavior

- **GO**: Writes a GO command to the CmdRingBuffer in SHM. Nav process picks it up on next 20Hz poll cycle (within 50ms). Robot starts searching/navigating.
- **STOP**: Writes a STOP command to the CmdRingBuffer. Robot stops immediately. Navigation state resets to IDLE.
- **CAP**: Saves the current camera frame as a PNG to `/root/captures/cap_YYYYMMDD_HHMMSS.png`. Button turns green on success, red on failure, resets after 1.5s.
- **SETTINGS**: Switches the Gtk.Stack to the settings view (same window, no separate dialog). Press BACK or ESC to return.
- **QUIT**: Sends STOP to motors, calls `state.request_shutdown()`, stops GTK main loop, all processes clean up.

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
| `+` / `=` | Zoom in radar (reduce range) |
| `-` | Zoom out radar (increase range) |
| `Q` | Quit application |
| `ESC` | Quit application (or return from settings) |

All keyboard events are captured by the main window's `key-press-event` handler.

---

## 8. Settings Window

A fullscreen settings view (auto-sized to screen, fallback 1024x600) with **5 main tabs** and sub-tabs inside. Opened via Gtk.Stack switch (not a separate dialog). All changes take effect **immediately** and are **auto-saved** to disk after 2 seconds of inactivity (debounced). Settings persist across restarts via `~/.config/target_nav/calibration.json`.

Header bar: **BACK** button (left), **"Robot Settings"** title (center), **SAVE ALL** button (right).

### Tab 1: Navigate

Sub-tabs: **Speed**, **Target**, **Approach**.

| Sub-tab | Parameters |
|---------|------------|
| Speed | Linear Speed (0.05–0.60, default 0.20 m/s), Min Speed (0.03–0.30, default 0.10 m/s), Angular Speed (0.10–1.50, default 0.40 rad/s) |
| Target | Approach Distance (0.0–2.0, default 0.22 m), Lost Timeout (0.5–30.0, default 3.0 s) |
| Approach | Obstacle Distance (0.05–2.0, default 0.22 m), Obstacle Slowdown Distance (0.1–3.0, default 0.45 m) |

### Tab 2: Search

Sub-tabs: **Scan**, **Spiral**.

| Sub-tab | Parameters |
|---------|------------|
| Scan | Search Timeout (5.0–120.0, default 30.0 s), Search Angular Speed (0.05–2.0, default 0.50 rad/s) |
| Spiral | Spiral Enabled (toggle, default On), Initial Radius (0.1–5.0, default 0.3 m), Max Radius (0.5–10.0, default 2.0 m), Growth Rate (0.01–1.0, default 0.15 m/rev), Linear Speed (0.01–1.0, default 0.10 m/s), Angular Speed (0.01–2.0, default 0.25 rad/s), Timeout (5.0–120.0, default 45.0 s) |

Blind approach parameters are also in this tab: Entry Distance (0.1–2.0, default 0.35 m), Approach Speed (0.01–0.5, default 0.10 m/s), Timeout (1.0–30.0, default 10.0 s), LiDAR Stop (0.05–1.0, default 0.15 m), Arrival Margin (0.01–1.0, default 0.10 m).

### Tab 3: Sensors

Sub-tabs: **Detection**, **Calibration**, **Mounting**, **DRP-AI**.

| Sub-tab | Parameters |
|---------|------------|
| Detection | Confidence Threshold (0.05–0.95, default 0.30), Detection Memory (0.3–5.0, default 4.0 s) |
| Calibration | Auto-calibrate button (place target at known distance), manual ref box height/distance, target dimensions (height/width), camera FOV (10–170°, default 60°), distance reference point (center/front/camera/lidar) |
| Mounting | Robot dimensions (length/width/height), LiDAR position (X/Y/Z) + yaw (0–360°, default 180°), Camera position (X/Y/Z) + yaw (-180–180°, default 0°), robot diagram |
| DRP-AI | C++ confidence threshold (0.10–0.95, default 0.20), NMS threshold (0.10–0.95, default 0.45), Max detections (1–20, default 1), DRP core frequency, AI-MAC frequency, Restart DRP-AI button |

### Tab 4: Radar

Flat page (no sub-tabs). Controls the radar view panel appearance.

| Parameter | Range | Default | Effect |
|-----------|-------|---------|--------|
| Radar Range | 1–20 | 5 | Radar view radius in meters |
| Laser Point Size | 1–5 | 2 | LiDAR dot size in pixels |
| Arrow Length | 8–50 | 20 | Heading arrow length in pixels |
| Show Grid | Toggle | On | Show/hide distance circles |
| Show Nav Target | Toggle | On | Show/hide navigation target diamond |

Includes a **RESET MAP DEFAULTS** button.

### Tab 5: Tools

Sub-tabs: **Drive**, **Motors**, **System**.

| Sub-tab | Content |
|---------|---------|
| Drive | Hold-to-move motor test buttons (Forward, Backward, Left, Right, Turn L, Turn R) with configurable speed. Motors run while button is held, stop on release. |
| Motors | Arduino commands: CALIBRATE (`CALIB`), SYNC, READ, RESET ENC. Reset Odometry button. |
| System | Reset All Defaults button (resets all navigation, map, and detection parameters to factory defaults). |

### Auto-Save Behavior

All parameter changes are automatically saved to disk:
1. User adjusts a slider or toggle
2. A 2-second debounce timer starts (resets on each new change)
3. After 2 seconds of no changes, `save_calibration()` writes to `~/.config/target_nav/calibration.json`
4. On next startup, all saved parameters are restored with type-safe coercion

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
└── Gtk.Stack
    ├── "main" → VBox
    │   ├── DrawingArea (map + camera rendering via Cairo)
    │   └── HBox (control bar, 50px min height)
    │       ├── Button "GO TO TARGET" (suggested-action, 180x45)
    │       ├── Button "STOP" (destructive-action, 180x45)
    │       ├── Button "CAP" (60x45)
    │       ├── Label (status, Pango markup, expandable)
    │       ├── Button "QUIT" (quit-btn, 100x45)
    │       └── Button "SETTINGS" (settings-btn, 120x45)
    └── "settings" → (content from SettingsWindow)

SettingsWindow (Gtk.Window, fullscreen auto-sized)
└── VBox
    ├── HBox (header: BACK, title, status, SAVE ALL)
    └── Notebook (5 tabs)
        ├── Tab "Navigate" → Sub-notebook (Speed, Target, Approach)
        ├── Tab "Search" → Sub-notebook (Scan, Spiral)
        ├── Tab "Sensors" → Sub-notebook (Detection, Calibration, Mounting, DRP-AI)
        ├── Tab "Radar" → VBox (range, point size, grid, nav target toggles)
        └── Tab "Tools" → Sub-notebook (Drive, Motors, System)
```
