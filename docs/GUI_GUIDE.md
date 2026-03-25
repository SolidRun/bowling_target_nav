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
│  │   ■ Robot (green)      │  │   ┌────┐                      │ │
│  │   · Laser (cyan)       │  │   │pin│ 0.45m  12°            │ │
│  │   ◇ Target (red)       │  │   └────┘                      │ │
│  │   ▲ Camera FOV (green) │  │                                │ │
│  │                        │  │              v=0.12 m/s  w=5°/s│ │
│  │  Robot: (1.2, 0.5) 45° │  │  Detection: Target: 0.45m, 12° │ │
│  └────────────────────────┘  └────────────────────────────────┘ │
│                                                                  │
│  [GO TO TARGET] [STOP] [CAP] NAVIGATING 0.45m  [QUIT] [SETTINGS] │
└─────────────────────────────────────────────────────────────────┘
```

**Technology**: GTK3 with Cairo 2D graphics, Pango for text rendering.

**Refresh Rate**: 10 FPS (100ms timer via `GLib.timeout_add`) — saves CPU on embedded.

---

## 2. Main Window Layout

The window is divided into three vertical sections:

### Title Bar (Top 50px)
- **"V2N Target Control"** — Blue (#58A6FF), 20px font
- **Subtitle** — `"{detector_mode} + LiDAR + Navigation"` (green when DRP-AI active, gray otherwise), 14px, offset 280px right

### Content Area (Middle, fills remaining space)
- Split 50/50 into left (Map) and right (Camera) panels
- 10px margin around edges, 10px gap between panels
- Radar panel: dark background (#0D0D14) with gray border (#303C3D)
- Camera panel: dark background (#161B22), no border

### Control Bar (Bottom, 50px min height)
- Horizontal box with 20px spacing, 20px margins
- Buttons: GO, STOP, CAP, (expandable status label), QUIT, SETTINGS

---

## 3. Radar Panel (Left)

**Header**: Rendered inline by `draw_map_panel()` — no separate header widget.

### What's Displayed

| Element | Color | Description |
|---------|-------|-------------|
| Robot body | Green rectangle + wheels | True-to-scale from physical dimensions |
| Front edge | Bright green + "F" label | Shows robot forward direction |
| Forward arrow | Yellow | Heading arrow (configurable length) |
| Laser dots | Cyan (#00FF99) | Live LiDAR scan points (rectangles, faster than arcs) |
| Nav target | Red diamond | Target position in odom frame with distance label |
| Camera FOV | Green wedge | Camera field-of-view cone with "CAM" label |
| LiDAR marker | Red circle | LiDAR position with "LDR" label and 360° scan ring |
| Grid | Gray circles | Distance circles at 1m increments (when show_grid=True) |
| Sensor offsets | Dashed lines | Camera (green) and LiDAR (red) offset from base_link |

### How Radar Rendering Works

1. Robot is always at center (ego-centric view, rotates with robot)
2. LiDAR points are plotted in robot-relative coordinates (X=forward→screen up, Y=left→screen left)
3. Points inside robot body footprint are filtered out (self-reflections)
4. Navigation target (if any) is drawn as a red diamond with distance label from camera position

### Diagnostic Footer (Bottom of Radar Panel)

Two-line display below the radar:

1. **Robot pose** (green): `Robot: (1.23, 0.45) 67°`
2. **Scan info** (light blue): `Scan: 5.5Hz (360pts)  |  Range: 5m`

### Radar Zoom

The radar range can be adjusted via +/- buttons (28x28px) in the top-right corner of the radar panel, or keyboard shortcuts (+/-/=). Steps: 1, 2, 3, 5, 8, 10, 15, 20 meters. The Settings slider limits to 1–10, but zoom buttons allow up to 20m.

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

**CLIP indicator**: When a detection bounding box touches within 5px of any frame edge, it is marked as "clipped". This means the target extends beyond the visible frame, making the vision distance estimate unreliable (the bbox height is truncated). When CLIP is active, the navigator prefers LiDAR distance over camera distance, and the info bar shows an orange "CLIP" label next to the confidence percentage.

### State Badge Colors (Camera Panel)

These are Cairo RGB values from `camera_panel.py`, rendered as semi-transparent rectangles in the top-right corner of the camera feed:

| State | RGB | Hex | Display |
|-------|-----|-----|---------|
| NAVIGATING | (0.137, 0.533, 0.212) | #238836 | Dark green |
| SEARCHING | (0.886, 0.686, 0.0) | #E1AE00 | Gold |
| SPIRAL SEARCH | (0.878, 0.565, 0.251) | #E09040 | Darker orange |
| BLIND APPROACH | (0.902, 0.557, 0.149) | #E68E25 | Orange |
| ARRIVED | (0.122, 0.435, 0.918) | #1F6EEA | Blue |
| IDLE | (0.35, 0.38, 0.42) | #59606B | Gray |
| ERROR | (0.855, 0.212, 0.200) | #DA3633 | Red |

> Note: The badge displays `nav_state.replace("_", " ")`, so "BLIND_APPROACH" appears as "BLIND APPROACH".

### Info Bar (Bottom, 80px)

Three equal-width columns with dark background (#1C2129):

- **Col 1 — Camera distance** (large green #33FF80): `0.45m` + small `vision +12°`. Shows `--.-m` when no target.
- **Col 2 — LiDAR distance** (large light blue #40BBFF): `0.42m` + small `lidar` when matched, `sensor` otherwise. Shows `--.-m` when no match.
- **Col 3 — Confidence** (large, color-coded): `85%` (green >=70%, yellow 50-70%, red <50%). Shows `CLIP` indicator (orange #FF4C00) when bbox touches frame edge. Speed/omega line always shown: light blue when moving (speed>0.001 or wz>0.01), gray when stopped.
- **Bottom row**: Detector mode string (small gray), e.g. "DRP-AI Stream"

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
- **CAP**: Saves the current camera frame as a PNG to `/root/captures/cap_YYYYMMDD_HHMMSS.png` (pure Python PNG encoder, no OpenCV). Button label changes to "OK" (green) on success, "FAIL" (red) on failure, resets to "CAP" after 1.5s.
- **SETTINGS**: Switches the Gtk.Stack to the settings view (same window, no separate dialog). Press BACK or ESC to return.
- **QUIT**: Sends STOP to motors, calls `state.request_shutdown()`, stops GTK main loop, all processes clean up.

---

## 6. Status Bar

Located between CAP and QUIT buttons, the status label shows navigation state with color-coded Pango markup. It expands to fill available space.

### Status Format

```
NAVIGATING  Sensor: 0.45m  Dist: 0.42m  0.12 m/s  ! OBS 0.30m
```

### State Indicators (Pango markup colors)

| State | Color | Additional Info |
|-------|-------|-----------------|
| IDLE | Gray (#6e7681) | "Press GO to start" |
| NAVIGATING | Green (#50fa7b) | Sensor distance, map distance, speed |
| SEARCHING | Gold (#f0d050) | Elapsed time, lost time |
| SPIRAL_SEARCH | Orange (#e09040) | Elapsed time, lost time, speed |
| BLIND_APPROACH | Orange (#ffb347) | "Dead-reckoning", map distance, lost time |
| ARRIVED | Blue (#69b4ff) | "at target!" |
| ERROR | Red (#ff6b6b) | Error message |

### Status Additions

- **Map distance** (color-coded): Green (#50fa7b) if <0.20m, yellow (#f0d050) if <0.50m, blue (#79c0ff) otherwise
- **Obstacle warning** (red #ff6b6b): `! OBS 0.30m` when `obstacle_ahead == True`
- **Speed** (light blue #79c0ff): `0.12 m/s` when speed > 0.01 m/s

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
| Speed | Forward Speed (0.05–0.40, default 0.20 m/s), Min Speed (0.03–0.20, default 0.10 m/s), Rotation Speed (0.10–1.00, default 0.40 rad/s) |
| Target | Stop Distance (0.03–0.50, default 0.22 m), Obstacle Distance (0.10–0.50, default 0.22 m), Slowdown Zone (0.20–1.00, default 0.45 m), Target Lost Timeout (1.0–15.0, default 3.0 s) |
| Approach | Entry Distance (0.15–1.00, default 0.35 m), Approach Speed (0.03–0.20, default 0.10 m/s), Timeout (3.0–30.0, default 10.0 s), LiDAR Stop (0.05–0.30, default 0.15 m), Arrival Margin (0.03–0.30, default 0.10 m) |

### Tab 2: Search

Sub-tabs: **Scan**, **Spiral**.

| Sub-tab | Parameters |
|---------|------------|
| Scan | Scan Speed (0.05–1.00, default 0.50 rad/s), Scan Timeout (10.0–90.0, default 30.0 s) |
| Spiral | Spiral Enabled (toggle, default On), Start Radius (0.1–1.0, default 0.3 m), Max Radius (0.5–5.0, default 2.0 m), Growth Rate (0.05–0.50, default 0.15 m/rev), Spiral Speed (0.05–0.30, default 0.10 m/s), Turn Speed (0.1–0.5, default 0.25 rad/s), Spiral Timeout (10.0–120.0, default 45.0 s) |

### Tab 3: Sensors

Sub-tabs: **Detection**, **Calibration**, **Mounting**, **DRP-AI**.

| Sub-tab | Parameters |
|---------|------------|
| Detection | Confidence Threshold (0.05–0.95, default 0.30), Detection Memory (0.3–5.0, default 4.0 s) |
| Calibration | Auto-calibrate button (place target at known distance, enter known dist 0.1–5.0m), manual ref box height/distance spinners, target dimensions Height (5–100cm) and Width (2–50cm), camera FOV (30–120°, default 60°), distance reference point (Center/Front edge/Camera/LiDAR) |
| Mounting | Robot dimensions (length 10–60cm, width 10–60cm, height 5–40cm), LiDAR position (Fwd ±20cm, Lat ±20cm, Ht 0–30cm) + yaw (-360–360°, default 180°, presets: 0/±45/±90/±135/±180), Camera position (Fwd ±20cm, Lat ±20cm, Ht 0–30cm) + yaw (-360–360°, default 0°), live robot diagram |
| DRP-AI | AI Confidence (0.10–0.95, default 0.20), NMS Threshold (0.10–0.95, default 0.45), DRP Core frequency dropdown, AI-MAC frequency dropdown, Apply & Restart DRP-AI button |

### Tab 4: Radar

Flat page (no sub-tabs). Controls the radar view panel appearance.

| Parameter | Range | Default | Effect |
|-----------|-------|---------|--------|
| Radar Range | 1–10 | 5 | Radar view radius in meters |
| Laser Point Size | 1–5 | 2 | LiDAR dot size in pixels |
| Arrow Length | 10–40 | 20 | Heading arrow length in pixels |
| Show Grid | Toggle | On | Show/hide distance circles |
| Show Nav Target | Toggle | On | Show/hide navigation target diamond |

Includes a **RESET MAP DEFAULTS** button.

### Tab 5: Tools

Sub-tabs: **Drive**, **Motors**, **System**.

| Sub-tab | Content |
|---------|---------|
| Drive | 12-button direction grid (FWD, BWD, LEFT, RIGHT, FL, FR, BL, BR diagonals, TL/TR rotation, STOP). PWM speed slider (20–255), ticks slider (100–17190, ~17.19 ticks/mm) with live distance label. Buttons send firmware position commands directly (e.g. `FWD,100,1719`). Firmware handles PID, acceleration, deceleration, and auto-stop on completion (`DONE`). |
| Motors | Single motor test: PWM slider (0–200), Forward/Reverse radio, duration spinner (0.5–5.0s), per-motor buttons (FL, FR, RL, RR). Sends `TMOTOR,id,pwm` to Arduino. Auto-stops after duration. |
| System | Reset Odometry button. Motor calibration with 60s countdown timer: START CALIBRATION (`CALIB`), ABORT (`STOP`). |

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
