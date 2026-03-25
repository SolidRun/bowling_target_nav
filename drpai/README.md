# DRP-AI YOLO Detection Binary

> C++ application that runs YOLOv8 inference on the Renesas DRP-AI hardware accelerator (RZ/V2N).

---

## Table of Contents

1. [Overview](#1-overview)
2. [Operating Modes](#2-operating-modes)
3. [Building](#3-building)
4. [Deploying](#4-deploying)
5. [Configuration](#5-configuration--configini)
6. [Shared Memory Protocols](#6-shared-memory-protocols)
7. [Source Files](#7-source-files)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Overview

The `app_yolo_cam` binary runs on CPU Core 3 of the RZ/V2N SoC. It:

1. Captures camera frames from `/dev/video0` (V4L2, with GStreamer fallback)
2. Preprocesses frames (letterbox resize to 640x640, BGR→RGB)
3. Runs YOLOv8 inference on DRP-AI hardware accelerator (not CPU)
4. Post-processes results (DFL decode, confidence filter, soft-NMS)
5. Estimates distance and angle using calibrated pinhole model
6. Draws bounding boxes, labels, crosshairs, and distance on the frame
7. Writes annotated frames to `/dev/shm/v2n_camera` (GUI reads via mmap)
8. Writes detection structs to `/dev/shm/v2n_detections` (Python camera_worker reads)
9. Reads calibration updates from `/dev/shm/v2n_calibration` (Python writes)

**Model:** YOLOv8 (ultralytics 8.4.7), single-class (bowling-pin), input 640x640, DFL cut-heads format with 8400 anchors.

---

## 2. Operating Modes

### Stream mode (default, used by camera_node.py)

The C++ binary owns the camera and runs continuously. Python reads results from shared memory.

```
Camera ──► C++ capture ──► DRP-AI inference ──► SHM frames + SHM detections
                                                      │              │
                                                 GUI reads      camera_worker reads
                                                 (mmap)          (struct)
```

- Fastest mode: zero-copy frame sharing via mmap (~921KB per frame saved vs pipe)
- C++ handles: capture, inference, annotation, distance estimation
- Python handles: temporal tracking, shape filtering, Kalman fusion

### Pipe mode (fallback, used when stream mode fails)

Python owns the camera and sends frames to C++ via stdin. C++ returns detection JSON via stdout.

```
Python camera ──► stdin ──► C++ inference ──► stdout JSON ──► Python parsing
```

- Slower: full frame transfer per inference (~921KB via pipe)
- Used when: stream mode initialization fails (camera busy, SHM error)

---

## 3. Building

Requires the Renesas DRP-AI TVM Docker container with the cross-compilation SDK.

```bash
# Inside Docker container:
cd drpai
chmod +x build.sh package.sh

./build.sh          # Cross-compile for V2N (ARM Cortex-A55)
./package.sh        # Package binary + model + config into deploy/
```

### What build.sh does

1. Unsets `LD_LIBRARY_PATH` to prevent host library contamination
2. Auto-detects SDK environment (`environment-setup-*poky-linux` under `/opt/`)
3. Sets `TVM_ROOT` for the DRP-AI TVM compiler runtime
4. Runs CMake with the ARM cross-compilation toolchain (`toolchain/runtime.cmake`)
5. Builds with `-O3 -mtune=cortex-a55`

### What package.sh does

1. Copies `app_yolo_cam` binary to `deploy/`
2. Copies `config.ini` and `labels.txt`
3. Copies shared libraries from `lib/`
4. Searches for the model in `../drpai_model/`, `../../drpai_model/`, or `../model/`
5. Copies only runtime model files (`deploy.so`, `deploy.json`, `deploy.params`), skips compilation artifacts

Output: `drpai/deploy/` ready to copy to V2N.

---

## 4. Deploying

```bash
# From PC:
scp -r drpai/deploy/* root@192.168.50.1:/home/root/deploy/

# On V2N:
chmod +x /home/root/deploy/app_yolo_cam
cp /home/root/deploy/lib/*.so /usr/lib/
ldconfig
```

Or use `scripts/setup/v2n_setup.sh` which handles this automatically.

---

## 5. Configuration — config.ini

Place `config.ini` next to the binary. Parameters override compile-time defaults from `define.h`.

**Priority:** CLI flags > config.ini > compile-time defaults (define.h)

```ini
# --- Detection ---
conf_threshold = 0.50        # Minimum confidence (0.0–1.0). Default compile-time: 0.80
nms_threshold = 0.45         # Soft-NMS overlap threshold (0.0–1.0)
max_detections = 1           # Max bounding boxes per frame

# --- Class Names ---
class_names = bowling-pin    # Comma-separated, overrides labels.txt
display_label = Pin          # Short label for overlay ("Pin 85%")

# --- Camera ---
# camera_device = /dev/video0
# camera_width = 640
# camera_height = 480
# warmup_frames = 10         # Frames to discard on startup (auto-exposure)

# --- Model ---
# model_dir = model           # Subdirectory containing TVM model files

# --- Display ---
# display_mode = wayland      # "wayland" or "console" (headless)

# --- DRP-AI Frequency ---
# drp_max_freq = 2            # DRP core clock divider
# drpai_freq = 5              # AI-MAC clock divider

# --- Threading ---
# wait_time = 1000            # Thread polling interval (microseconds)
```

### Class names loading priority

1. CLI argument `--labels <file>`
2. `labels.txt` in the deploy directory
3. `class_names=` in config.ini (comma-separated)
4. Compile-time `CLASS_NAMES[]` in define.h

### DRP-AI frequency reference

These are Renesas clock divider indices, not raw frequencies. Lower index = higher frequency = faster inference but more power.

| Level | DRP Core | AI-MAC |
|-------|----------|--------|
| 1 | — | 1 GHz |
| 2 | 420 MHz | 1 GHz |
| 3 | 315 MHz | 630 MHz |
| 4 | 252 MHz | 420 MHz |
| 5 | 210 MHz | 315 MHz |
| 127 | 9.8 MHz (min) | 10 MHz (min) |

**Production defaults:** `drp_max_freq=2` (420 MHz), `drpai_freq=5` (315 MHz).
**Maximum performance:** `drp_max_freq=2` (420 MHz), `drpai_freq=1` (1 GHz).

---

## 6. Shared Memory Protocols

All shared memory files live under `/dev/shm/` (tmpfs, never touches SD card). All multi-byte values are **little-endian**.

### `/dev/shm/v2n_camera` — Annotated BGR frames

C++ writes, GUI reads (via mmap in `main.py`).

**Header (24 bytes):**

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 4 | uint32 | width | Frame width in pixels |
| 4 | 4 | uint32 | height | Frame height in pixels |
| 8 | 8 | uint64 | sequence | Monotonic counter (written LAST as commit marker) |
| 16 | 8 | uint64 | timestamp | Microseconds since steady_clock epoch |

**Pixel data (offset 24, width × height × 3 bytes):**

BGR format, row-major, uint8. Includes all detection overlays (bboxes, labels, crosshairs) pre-rendered by C++.

**Torn-write detection:**

The writer updates width, height, timestamp, and pixel data first, then writes the sequence number last (with a memory barrier). The reader reads the sequence before and after the pixel data — if they differ, the frame was partially written and is discarded.

### `/dev/shm/v2n_detections` — Detection structs

C++ writes, Python `DetectionShmReader` reads (in `shm_reader.py`).

**Total size:** 48 (header) + 20 × 48 (entries) = **1008 bytes**.

**Header (48 bytes):**

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 4 | uint32 | n_detections | Number of valid entries (0–20). Written LAST as commit marker |
| 4 | 4 | uint32 | frame_seq | Monotonic frame sequence |
| 8 | 4 | float | inference_ms | DRP-AI inference latency in milliseconds |
| 12 | 4 | float | ref_box_height | Echoed calibration: reference bbox height (pixels) |
| 16 | 4 | float | ref_distance | Echoed calibration: reference distance (meters) |
| 20 | 4 | float | camera_fov | Echoed calibration: horizontal FOV (degrees) |
| 24–47 | 24 | — | padding | Reserved |

**Per-detection entry (48 bytes each, starting at offset 48):**

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 4 | float | x1 | Bbox left edge (pixels) |
| 4 | 4 | float | y1 | Bbox top edge (pixels) |
| 8 | 4 | float | x2 | Bbox right edge (pixels) |
| 12 | 4 | float | y2 | Bbox bottom edge (pixels) |
| 16 | 4 | float | confidence | Detection confidence (0.0–1.0) |
| 20 | 4 | float | distance | Estimated distance in meters (0.0 if uncalibrated) |
| 24 | 4 | float | angle | Horizontal angle from frame center (degrees) |
| 28 | 4 | int32 | class_id | Class index (0 = bowling-pin) |
| 32 | 1 | uint8 | bbox_clipped | 1 if bbox touches top/bottom frame edge, 0 otherwise |
| 33–47 | 15 | — | padding | Reserved |

**Commit protocol:** C++ writes all entries first, then writes the header fields, then `frame_seq`, then `n_detections` last (with memory barrier). The reader checks `frame_seq` before and after reading entries — if they differ, the data is discarded.

### `/dev/shm/v2n_calibration` — Calibration parameters

Python `CalibrationShmWriter` writes (in `camera_worker.py`), C++ reads (polls every ~2s).

**Total size: 16 bytes.**

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 4 | float | ref_box_height | Reference bounding box height in pixels |
| 4 | 4 | float | ref_distance | Reference distance in meters |
| 8 | 4 | float | camera_fov | Horizontal field of view in degrees |
| 12 | 4 | uint32 | update_seq | Incremented by Python when calibration changes |

C++ uses these values for distance estimation on the overlay. When the user changes calibration in the GUI, the updated values propagate to C++ within ~2 seconds.

---

## 7. Source Files

| File | Purpose |
|------|---------|
| `src/main.cpp` | Entry point, config parsing, multi-threaded capture/inference/display pipeline, SHM creation and writing, calibration polling, distance estimation |
| `src/define.h` | Compile-time constants: model dimensions (640x640), thresholds, camera defaults, frequency levels, display settings |
| `src/drpai_inference.cpp` | DRP-AI hardware API: model loading, frequency setting via ioctl, inference execution, single and multi-output model support |
| `src/yolo_postprocess.cpp` | YOLOv8 DFL decoder, soft-NMS, confidence filtering, class scoring for both decoded and cut-heads model formats |
| `src/camera_capture.cpp` | V4L2 camera capture with GStreamer fallback, buffer minimization (1 frame), warmup frames |
| `src/display.cpp` | Frame annotation (bboxes, labels, crosshairs, distance, FPS, CLIPPED indicator), Wayland auto-detection and GStreamer waylandsink output |
| `CMakeLists.txt` | Build configuration: target `app_yolo_cam`, links OpenCV, DRP-AI TVM runtime, pthread, rt |
| `toolchain/runtime.cmake` | Cross-compilation toolchain for ARM Cortex-A55 (aarch64) |
| `config.ini` | Runtime config (copied to `deploy/` by `package.sh`) |

---

## 8. Troubleshooting

### Binary won't start

```bash
# Check if binary exists and is executable
ls -la /home/root/deploy/app_yolo_cam

# Check shared libraries
ldd /home/root/deploy/app_yolo_cam

# If libraries missing:
cp /home/root/deploy/lib/*.so /usr/lib/
ldconfig
```

### Inference is slow

1. Check DRP-AI frequency: try `drp_max_freq=2`, `drpai_freq=1` in config.ini
2. Check CPU usage on Core 3: `taskset -c 3 top`
3. Check kernel logs: `dmesg | grep -i drp`
4. Verify model format: decoded format is faster than cut-heads

### No detections

1. Verify camera: `v4l2-ctl --device=/dev/video0 --stream-mmap --stream-count=1`
2. Lower confidence: set `conf_threshold = 0.20` in config.ini
3. Check model exists: `ls /home/root/deploy/model/sub_*`
4. Check class names: `cat /home/root/deploy/labels.txt`
5. Check inference is running: look for `inference_ms` in detection SHM header

### Frame not appearing in GUI

1. Check SHM exists: `ls -la /dev/shm/v2n_camera`
2. Check binary is running: `pgrep -f app_yolo_cam`
3. Check SHM size is correct: `stat /dev/shm/v2n_camera` (should be 24 + 640×480×3 = 921624 bytes)
4. Restart: `pkill -9 -f app_yolo_cam` then relaunch GUI

### Calibration not updating

1. Check calibration SHM: `ls -la /dev/shm/v2n_calibration`
2. Verify Python is writing: the `update_seq` field (offset 12) should increment when you change settings
3. C++ polls every ~2 seconds — wait and check the overlay distance labels

### Stale shared memory after crash

```bash
# Clean all SHM files
rm -f /dev/shm/v2n_camera /dev/shm/v2n_detections /dev/shm/v2n_calibration
# Then restart the GUI (which recreates SHM)
```
