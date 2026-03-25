# Navigation Reference

> Complete reference for navigation algorithms, state machine, detection filtering, target tracking, and troubleshooting.

> **Cross-reference:** For system architecture (processes, SHM, data flow), see [ARCHITECTURE.md](ARCHITECTURE.md). For GUI controls and settings, see [GUI_GUIDE.md](GUI_GUIDE.md).

---

## Table of Contents

1. [Per-Tick Pipeline](#1-per-tick-pipeline)
2. [State Machine](#2-state-machine)
3. [State Details](#3-state-details)
4. [VFH Obstacle Avoidance](#4-vfh-obstacle-avoidance)
5. [Speed Ramp](#5-speed-ramp)
6. [Detection Filtering](#6-detection-filtering-false-positive-rejection)
7. [Target Tracking (Kalman Filter)](#7-target-tracking-kalman-filter)
8. [Arrival Detection](#8-arrival-detection)
9. [Sensor Offset Model](#9-sensor-offset-model)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. Per-Tick Pipeline

The Navigator runs a 20 Hz control loop (50 ms per tick). Each tick follows this pipeline:

```
DetShmReader ──► TargetTracker ──► navigate_to_target() ──► /cmd_vel
  (20 Hz)        (gate+Kalman)     (VFH + speed ramp)      (Twist)
```

Detailed per-tick flow:

```
1. Poll detections from struct SHM (DetShmReader)
2. TargetTracker.update(detections, lidar_points):
   ├── find_best_target() — pick highest-priority detection
   ├── Shape filter — reject wrong width/height ratio (§ 6, Layer 2)
   ├── Size-distance gate — reject wrong-sized bboxes (§ 6, Layer 3)
   ├── LiDAR matching — find LiDAR point at detection bearing (±10°)
   └── Kalman filter — smooth [distance, angle, d_velocity, a_velocity]
3. NavController.step():
   ├── If tracked target exists → navigate_to_target(target)
   ├── If target lost < 3.0s → keep navigating last known position
   ├── If target lost > 3.0s and close → BLIND_APPROACH
   ├── If target lost > 3.0s and far → SEARCHING
   └── If arrival confirmed (0.3s sustained) → ARRIVED
4. navigate_to_target():
   ├── Project target to odom frame via TF2 (or manual fallback)
   ├── Convert odom goal back to base_link for steering
   ├── Check arrival via multi-signal fusion (§ 8)
   ├── Compute speed via distance ramp (§ 5)
   └── Call direct_navigate() for VFH obstacle avoidance (§ 4)
5. Publish /cmd_vel (Twist) with speed caps and NaN protection
```

---

## 2. State Machine

```
                    ┌──────────────────────────────────────────┐
                    │              user GO command              │
                    ▼                                          │
               ┌─────────┐                              ┌──────────┐
     ┌────────►│  IDLE    │◄──── STOP (any state) ◄─────│ ARRIVED  │
     │         └────┬─────┘                              └──────────┘
     │              │ GO                                       ▲
     │              ▼                                          │
     │         ┌──────────────┐──── arrival confirmed ─────────┘
     │         │  NAVIGATING  │     (0.3s sustained)
     │         └──┬───────┬───┘
     │            │       │
     │   lost>3s  │       │  lost>3s
     │   + far    │       │  + close (<0.35m)
     │            ▼       ▼
     │     ┌───────────┐  ┌─────────────────┐
     │     │ SEARCHING  │  │ BLIND_APPROACH  │
     │     │ (360° scan)│  │ (dead-reckoning)│
     │     └─────┬──────┘  └───┬─────┬──────┘
     │           │             │     │
     │  360° done│    arrived ─┘     │ timeout/
     │  +spiral  │                   │ heading>45°
     │           ▼                   │
     │  ┌────────────────┐           │
     │  │ SPIRAL_SEARCH  │           │
     │  │ (Archimedean)  │           │
     │  └───────┬────────┘           │
     │          │ exhausted           │
     └──────────┴─────────────────────┘
```

**6 navigation states:** IDLE, NAVIGATING, SEARCHING, SPIRAL_SEARCH, BLIND_APPROACH, ARRIVED.

---

## 3. State Details

### IDLE

Waiting for user GO command. Robot is stopped. No navigation active.

- **Enter:** Application start, STOP command, search exhausted, or spiral exhausted.
- **Per tick:** Nothing. Motor commands are zero.
- **Exit:** User presses GO → NAVIGATING.

### NAVIGATING

Driving toward a detected bowling-pin using VFH obstacle avoidance. This is the primary active state where the robot sees the target and drives toward it.

- **Enter:** User presses GO, or target re-detected during BLIND_APPROACH/SEARCHING.
- **Per tick:**
    1. TargetTracker provides smoothed `(distance, angle)` in base_link frame.
    2. Navigator projects target to odom frame via TF2, stores as map goal.
    3. Speed ramp based on distance (see § 5):
       - Far (> 0.45m): `linear_speed` (default 0.20 m/s)
       - Near (0.22m–0.45m): linear ramp from `min_speed` (0.10) to `linear_speed` (0.20)
       - Very close (< 0.22m): `min_speed * 0.5` = 0.05 m/s (arrival creep)
    4. VFH obstacle avoidance (see § 4) steers around obstacles.
    5. Arrival check runs each tick (see § 8).
- **Exit conditions:**
    - Target lost > `lost_timeout` (3.0s) and last distance < 0.35m → BLIND_APPROACH
    - Target lost > `lost_timeout` (3.0s) and last distance >= 0.35m → SEARCHING
    - Arrival confirmed → ARRIVED

### SEARCHING (360° Scan)

Target lost. Robot rotates in place to re-acquire it.

The robot spins in the direction of the last known target angle — if the target was last seen to the left, it spins left first. This maximizes the chance of finding it quickly.

- **Enter:** Target lost for > `lost_timeout` (3.0s) while far from target.
- **Per tick:**
    1. Rotate at `search_angular_speed` (default 0.50 rad/s).
    2. Direction: toward the last known target angle (left if target was left, right if right).
    3. Track total rotation via yaw delta accumulation (wrapping-safe).
    4. Repulsive field: if near obstacles during rotation, drift away using a potential field force vector computed from nearby LiDAR points.
- **Exit conditions:**
    - Total rotation >= 360° and `spiral_search_enabled=True` → SPIRAL_SEARCH
    - Total rotation >= 360° and `spiral_search_enabled=False` → IDLE
    - `search_timeout` exceeded (default 30.0s) → IDLE
    - Target re-detected → NAVIGATING

### SPIRAL_SEARCH (Archimedean Spiral)

360° scan found nothing. Robot expands outward in a spiral pattern to cover more area, while oscillating the camera left-right to maximize detection coverage.

- **Enter:** 360° scan completed without finding target, spiral enabled.
- **Per tick:**
    1. Spiral equation: `r(θ) = initial_radius + growth_rate × θ / (2π)`
       - Default: starts at 0.3m radius, grows 0.15m per full revolution.
       - Example: after 3 revolutions, radius = 0.3 + 0.15 × 3 = 0.75m.
    2. Compute waypoint on spiral in map frame (centered on spiral start position).
    3. Drive toward waypoint at `spiral_linear_speed` (default 0.10 m/s).
    4. Camera sweep: sinusoidal yaw oscillation at 0.8 Hz so the camera scans left-right while the robot moves forward. This covers a wider detection angle than a fixed heading.
       - `angular_z = spiral_angular_speed × sin(elapsed × 0.8 × 2π)`
    5. VFH obstacle avoidance remains active during spiral (reduced speed factor 0.5).
- **Exit conditions:**
    - `spiral_timeout` exceeded (default 45.0s) → IDLE
    - `spiral_radius > spiral_max_radius` (default 2.0m) → IDLE
    - Target re-detected → NAVIGATING

### BLIND_APPROACH (Dead-Reckoning)

Target lost at close range. The robot is very close to the target but can no longer see it — either the target dropped below the camera's field of view, or it fell within the LiDAR's minimum range (0.15m). The robot dead-reckons to the last known odom-frame position using wheel odometry.

**Why this exists:** At close range (< 0.35m), the target often fills most of the camera frame and then disappears below it. Without blind approach, the robot would enter SEARCHING and spin away from a target that's right in front of it. Instead, it drives the remaining few centimeters using odometry.

- **Enter:** Target lost for > `lost_timeout` AND last distance < `blind_approach_entry_distance` (0.35m).
- **What's stored on entry:**
    - Robot pose at entry time (from odometry).
    - Target position in odom frame (from last camera detection → TF2 projection).
    - Starting distance to target.
- **Per tick:**
    1. Compute remaining distance from current robot pose to stored odom-frame goal.
    2. Compute heading error: `desired_heading - robot_theta` (wrapped to ±π).
    3. Speed ramp: `speed = blind_approach_speed × min(1.0, remaining / 0.30m)`
       - Ramps down as robot approaches the goal, preventing overshoot.
    4. Holonomic drive: `vx = speed × cos(heading_error)`, `vy = speed × sin(heading_error)`
       - This keeps the robot pointed at the goal while driving holonomically.
    5. Heading correction: `wz = 0.5 × heading_error`, clamped to ±0.2 rad/s.
- **Exit conditions (checked each tick, in order):**
    1. **Arrived:** Map distance ≤ `arrival_margin` (0.10m) → ARRIVED
    2. **Timeout:** Elapsed time > `blind_approach_timeout` (10.0s) → SEARCHING
    3. **LiDAR obstacle:** Front obstacle < `blind_approach_lidar_stop` (0.15m):
       - If close to goal (< arrival_margin × 3 = 0.30m) → ARRIVED (obstacle IS the target)
       - Else → SEARCHING (obstacle is something else)
    4. **Heading diverged:** Heading error > 45° → SEARCHING (target probably not ahead anymore, odometry drifted)
    5. **Stale pose:** Odometry stuck at (0,0,0) → SEARCHING (pose invalid)
    6. **Target re-detected:** Camera sees target again → NAVIGATING

### ARRIVED

Target reached. Robot is stopped.

- **Enter:** Arrival confirmed by multi-signal temporal fusion (see § 8).
- **Per tick:** Zero velocity. Motors stopped. Obstacle flag cleared.
- **Exit:** User presses GO → NAVIGATING (starts new navigation session).

---

## 4. VFH Obstacle Avoidance

The VFH (Vector Field Histogram) algorithm finds obstacle-free gaps in LiDAR data and steers through them. This implementation is adapted for holonomic mecanum drive — the robot can strafe sideways through a gap while independently rotating to keep the camera pointed at the target.

```
LiDAR points ──► Polar histogram ──► Find gaps ──► Pick best gap ──► Steer
  (front 180°)    (36 bins × 10°)    (body width)  (closest to target) (commit 1.5s)
```

### Step 1 — Polar histogram

The front 180° of LiDAR scan is divided into 36 angular bins (10° each). For each LiDAR point:
- If behind the robot (angle > 90° from forward): ignored.
- If closer than 0.03m: ignored (sensor noise floor).
- Otherwise: the closest obstacle distance is recorded in the corresponding bin.

### Step 2 — Gap finding

A "gap" is a contiguous run of bins where ALL obstacles are farther than `obstacle_distance + 0.10m` (clearance margin). The gap must be wide enough for the robot body to pass through:
- Required angular width: `2 × atan(robot_half_width + 0.03m, clearance_distance)`
- This converts the physical robot width (0.13m half + 0.03m margin) to an angular requirement at the clearance distance.

### Step 3 — Gap selection

Among all valid gaps, the one whose center angle is closest to the target bearing is chosen. This minimizes deviation from the desired heading.

### Step 4 — Direction commitment

Once a direction (strafe left or strafe right) is chosen, the robot commits to it for a minimum time to prevent oscillation between nearby gaps:
- 1.5s commitment when target is NOT visible (VFH_AVOID_COMMIT_SECS)
- 0.5s commitment when target IS visible (VFH_COMMIT_TARGET_SECS) — shorter because the target provides heading feedback

### Step 5 — Holonomic steering

The mecanum drive advantage: the robot can simultaneously:
- Move forward through the gap (reduced speed: `linear_speed × 0.6`)
- Rotate to keep the camera pointed at the target (fused mode)

This is unlike differential-drive VFH where the robot must choose between moving and rotating.

### Step 6 — Emergency backup

If no passable gap exists:
- Reverse at `min_speed × 0.3` (very slow backup).
- If stuck for > 3.0s (VFH_STUCK_TIMEOUT_SECS), full stop to prevent damage.

### Corridor Checks

In addition to VFH gap-finding, the navigator checks four body corridors for close obstacles:

| Corridor | Region | Effect |
|----------|--------|--------|
| Front | x > robot_front_edge, within body width | If obstacle < `obstacle_distance`: trigger VFH |
| Rear | x < robot_rear_edge, within body width | If obstacle too close: block backward motion |
| Left | within body length, y > body left edge | If obstacle within 0.40m lateral depth: block left strafe |
| Right | within body length, y < body right edge | If obstacle within 0.40m lateral depth: block right strafe |

**Fail-safe:** If no LiDAR data is available or the scan is older than 2.0s (`LIDAR_STALE_TIMEOUT_SECS`), the robot treats this as a front obstacle at 0.0m distance — effectively stopping until fresh data arrives.

---

## 5. Speed Ramp

Speed is computed based on distance to target:

```
Speed (m/s)
  0.20 ┤─────────────────────────────────────────── linear_speed
       │                              ╱
  0.10 ┤────────────── min_speed ────╱
       │                            ╱
  0.05 ┤── creep (min × 0.5) ─────╱
       │
  0.00 ┼───────┼──────────┼───────┼──────────────
       0     0.22       0.45                  Distance (m)
           approach   slowdown
```

| Zone | Distance | Speed | Behavior |
|------|----------|-------|----------|
| Far | > 0.45m | `linear_speed` (0.20 m/s) | Full speed, VFH active |
| Near | 0.22m – 0.45m | Linear ramp: 0.10 → 0.20 m/s | Slowing down, VFH active |
| Arrival | < 0.22m | `min_speed × 0.5` = 0.05 m/s | Creep, arrival check active |

**Formula (near zone):**
```
ratio = (distance - approach_distance) / (slowdown_distance - approach_distance)
speed = min_linear_speed + ratio × (linear_speed - min_linear_speed)
```

**Obstacle slowdown (additional):** If obstacles are detected between `obstacle_distance` (0.22m) and `obstacle_slowdown_distance` (0.45m), speed is further reduced:
```
slow_factor = (min_front - obstacle_distance) / (slowdown_distance - obstacle_distance)
slow = max(0.3, min(1.0, slow_factor))
speed = speed × slow
```

---

## 6. Detection Filtering (False Positive Rejection)

The YOLO model is trained to detect bowling pins, but it can produce false positives on objects with similar visual features (bottle shapes, chair legs, vertical edges). Three layers of filtering reject these before they reach the navigator.

```
C++ DRP-AI output (raw bboxes)
        │
        ▼
Layer 1: C++ confidence + NMS ──► reject low-confidence and overlapping boxes
        │
        ▼
Layer 2: Python shape filter ──► reject wrong width/height ratio
        │
        ▼
Layer 3: Python size-distance gate ──► reject wrong size for measured distance
        │
        ▼
Temporal tracker (DetectionTracker) ──► smooth through DRP-AI flicker
        │
        ▼
Kalman tracker (TargetTracker) ──► LiDAR fusion + smoothed output
```

### Layer 1: C++ Confidence and NMS (in DRP-AI binary)

The C++ binary applies confidence threshold (`conf_threshold`, default 0.20) and Non-Maximum Suppression (`nms_threshold`, default 0.45) before outputting detections. These are intentionally low — the C++ side casts a wide net, and Python does the smart filtering.

### Layer 2: Python Shape Filter (in camera_worker.py)

Uses the known physical dimensions of the target to reject detections with wrong proportions.

**How it works:** A bowling pin has a known width/height ratio. If a detected bounding box has a very different ratio, it's probably not a bowling pin — it might be a chair leg (too thin) or a chair seat (too wide).

**Known target:** bowling pin — height 0.28m, width 0.08m.

**Ratio:** `width / height = 0.08 / 0.28 = 0.286`

**Tolerance:** ±60% → acceptable range: **[0.114, 0.457]**

| Object | Bbox w/h ratio | vs range [0.114, 0.457] | Result |
|--------|---------------|-------------------------|--------|
| Bowling pin | ~0.28 | within range | PASS |
| Bottle | ~0.28 | within range | PASS (similar shape) |
| Chair leg | ~0.07 | below 0.114 | REJECT (too thin) |
| Chair seat | ~1.25 | above 0.457 | REJECT (too wide) |
| Person | ~0.24 | below 0.114 (at full height) | REJECT |
| Door frame | ~0.05 | well below 0.114 | REJECT |

**Clipped bbox handling:** When the bbox touches the frame edge (within 5px), the height is truncated and the ratio is unreliable. In this case, only the width is checked: `expected_width = ref_box_height × (ref_distance / distance) × 0.286`. If the actual width exceeds 3× the expected, the detection is rejected.

**Limitation:** Objects with similar proportions to a bowling pin (like bottles or narrow posts) will pass the shape filter. The YOLO model itself must learn to distinguish these during training. The shape filter only catches gross shape mismatches — it is a physics-based backup, not a replacement for good training data.

### Layer 3: Python Size-Distance Gate (in target_tracker.py)

Uses the pinhole camera model to verify that the bounding box size makes physical sense at the measured distance. A real bowling pin should produce a predictable bbox height at any given distance.

**Pinhole model:** `expected_height = ref_box_height × (ref_distance / measured_distance)`

| Distance | Expected bbox height (at 180px ref, 1.0m ref dist) |
|----------|-----------------------------------------------------|
| 0.5m | 180 × (1.0 / 0.5) = **360px** |
| 1.0m | 180 × (1.0 / 1.0) = **180px** |
| 2.0m | 180 × (1.0 / 2.0) = **90px** |
| 3.0m | 180 × (1.0 / 3.0) = **60px** |

**Acceptance:** Actual bbox height must be between **0.3× and 3.0×** the expected height. This range is generous (accommodates noise, partial occlusion, non-ideal camera angles) but still rejects obvious mismatches. For example, a 20px bbox detected at 0.5m (expected 360px) would be rejected because 20 < 360 × 0.3 = 108.

**Width cross-check:** The actual width is also compared against `expected_height × target_ratio`. If width is < 0.3× or > 3.0× expected, the detection is rejected even if height passes.

### Why Not Rely on YOLO Training Alone?

The YOLO model is the primary classifier and handles most cases correctly. The shape filter and size gate are defense-in-depth layers needed because:

1. **Low confidence threshold:** The C++ side runs at confidence 0.20 to avoid missing real targets at long range or partial occlusion. This means occasional false positives slip through.
2. **Physics knowledge:** The model doesn't know the camera's calibration or the physical size of a bowling pin at a specific distance. The size gate uses this knowledge to catch false positives that look right to the model but are physically impossible.
3. **Different error types:** C++ catches random noise (Layer 1). Shape filter catches wrong proportions (Layer 2). Size gate catches wrong scale (Layer 3). Each layer handles a different failure mode.
4. **Training is not perfect:** Even well-trained models can be confused by novel objects in new environments. The physics-based filters provide a safety net that doesn't require retraining.

---

## 7. Target Tracking (Kalman Filter)

The TargetTracker sits between the filtered detections and the Navigator. It produces a smoothed, continuous target position even when DRP-AI detection flickers.

```
Filtered Det ──► LiDAR match ──► Kalman filter ──► TrackedTarget
                  (±10°, ±50%)    (4D state)        (smoothed)
```

### Kalman State Vector (4D)

| State | Meaning | Unit | Example |
|-------|---------|------|---------|
| `x[0]` | Distance to target | meters | 1.25 |
| `x[1]` | Bearing angle | radians | -0.15 (slightly right) |
| `x[2]` | Distance rate | m/s | -0.18 (approaching) |
| `x[3]` | Angle rate | rad/s | 0.02 (drifting left) |

### Process Model

Constant-velocity: the filter assumes the target moves at constant velocity between measurements.

```
distance_new = distance_old + d_velocity × dt
angle_new    = angle_old    + a_velocity × dt
```

This is a simplification — the target (bowling pin) is stationary, but the robot is moving. The velocity terms capture the apparent motion of the target relative to the robot.

### Measurement

Camera provides `[distance, angle]` at ~10-15 fps. The Kalman filter runs at 20 Hz (control loop rate), so it predicts between camera frames.

**Measurement noise (R matrix):**

| Source | Distance variance | Angle variance | When used |
|--------|------------------|----------------|-----------|
| Camera only | 0.04 m² (~0.20m std) | 0.003 rad² (~3° std) | No LiDAR match |
| LiDAR confirmed | 0.001 m² (~0.03m std) | 0.003 rad² (~3° std) | LiDAR match found |

When LiDAR confirms the detection, the distance noise is 40× lower — the filter trusts LiDAR distance much more than camera distance.

### Prediction (Coasting)

When no detection is available, the filter predicts the target position using the velocity model. This keeps the track alive through brief DRP-AI gaps (1-3 frames).

- Maximum prediction time: **3.0 seconds** (`predict_timeout`).
- After 3.0s without a detection, the track is dropped and the target is considered lost.
- Track confidence decreases with each prediction-only tick.

### LiDAR Matching

For each camera detection, the tracker searches for a confirming LiDAR obstacle:

1. **Angular filter:** Find all LiDAR points within ±10° of the detection bearing.
2. **Distance filter:** From those, find points within ±50% of the camera distance (or Kalman-predicted distance if active).
3. **Result:** If a match is found, use the closest LiDAR distance (more accurate than camera). If no match, use camera distance.

The LiDAR distance is measured from the camera position (not robot center) to match the camera distance convention.

---

## 8. Arrival Detection

Arrival uses multi-signal temporal fusion. The robot is considered arrived when the SMALLEST distance from ANY source stays within `approach_distance` (0.22m) for 0.3 seconds continuously.

```
                   ┌── Robot-frame distance (target_robot_xy from Kalman)
                   │
best = min ────────┼── Map-frame distance (odom goal vs robot pose)
                   │
                   └── Sensor distance (camera/LiDAR fusion)

if best ≤ 0.22m for 0.3s → ARRIVED
```

### Distance Sources

| Source | How computed | Reliable when |
|--------|-------------|---------------|
| Robot-frame | `sqrt(x_bl² + y_bl²)` from Kalman-tracked position | Camera is active |
| Map-frame | `sqrt((goal_x - robot_x)² + (goal_y - robot_y)²)` | Camera lost but goal stored |
| Sensor | Raw distance from camera/LiDAR fusion | Any detection available |

### Why Multiple Sources?

- **Camera close range:** At < 0.20m, the bounding box fills the frame and vision distance becomes unreliable. Map-frame distance (from odometry) provides a backup.
- **Camera lost:** During blind approach, only map-frame distance is available. Without it, the robot couldn't confirm arrival.
- **Optimistic fusion:** Taking the SMALLEST distance means if ANY source says "you're there", the confirmation timer starts. This avoids missed arrivals from single-sensor failures.

### Temporal Confirmation (0.3s)

**Why 0.3s and not longer?** At 0.15 m/s approach speed, the robot crosses the 0.22m arrival zone in ~1.5s. A 0.8s+ confirmation window would mean the robot passes through the zone before arrival is confirmed. 0.3s catches it reliably.

**Hysteresis:** The timer only resets when distance exceeds `approach_distance × 1.5` (0.33m). This prevents the timer from flickering on/off at the boundary due to measurement noise.

---

## 9. Sensor Offset Model

The camera and LiDAR are mounted at different positions on the robot. This creates a discrepancy between vision-estimated distance and LiDAR-measured distance.

```
        Camera (forward-facing, elevated)
           |
           |  camera_x = 0.15m forward from base_link
           |  camera_z = 0.10m above base_link
           |
      base_link ──── LiDAR
                      lidar_x = 0.12m forward
                      lidar_z = 0.15m above base_link
```

### Distance Conventions

- **Camera distance:** Measured from the camera position (not robot center) to the target, using the pinhole model. Less accurate but works at all ranges including the LiDAR blind zone.
- **LiDAR distance:** Measured from the LiDAR position to the nearest obstacle at the detection bearing. Accurate but has a 0.15m minimum range.
- **Both are converted to camera-centric distance** for consistency in the Kalman filter.

### Fusion Rule

The TargetTracker prefers LiDAR distance when available because it's more accurate:
- LiDAR measurement noise: 0.001 m² (σ ≈ 0.03m)
- Camera measurement noise: 0.04 m² (σ ≈ 0.20m)

When LiDAR doesn't match (no obstacle within ±10° and ±50% distance), camera distance is used alone.

### Reference Points

The distance estimator supports four reference points for computing distance:

| Reference | Distance measured from | Use case |
|-----------|----------------------|----------|
| center | base_link origin | Default geometry |
| front | Robot front bumper edge | "How far is it from the bumper?" |
| camera | Camera mounting position | Camera-centric tracking |
| lidar | LiDAR mounting position | LiDAR-centric measurement |

The chosen reference point affects the arrival distance threshold — e.g., "front" reference means the robot is arrived when the bumper is 0.22m from the target, not the center.

### Blind Zone

Below ~0.15m from the LiDAR (RPLidar A1 minimum range), only camera and odometry are available. The blind approach phase (§ 3, BLIND_APPROACH) handles navigation in this region using dead-reckoning.

---

## 10. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Robot overshoots target | Approach speed too high or arrival confirmation too slow | Reduce `linear_speed` (Settings → Navigate → Speed) or reduce `APPROACH_CREEP_FACTOR` in `navigator.py` |
| Robot oscillates near obstacles | VFH commit time too short | Increase `VFH_AVOID_COMMIT_SECS` in `navigator.py` (default 1.5s) |
| Robot gets stuck behind obstacle | No VFH gap wide enough | Check `VFH_CLEARANCE_MARGIN` (0.10m) and `obstacle_distance` in Settings |
| Target lost too quickly | Detection memory or lost timeout too short | Increase `detect_expiry` (Settings → Sensors → Detection) or `lost_timeout` (Settings → Navigate → Target) |
| Blind approach misses target | Odometry drift too high at close range | Reduce `blind_approach_arrival_margin` or increase `BLIND_LIDAR_ARRIVAL_FACTOR` in `navigator.py` |
| False detection (wrong object) | Confidence too low or shape filter too permissive | Increase confidence (Settings → Sensors → Detection) or tighten `RATIO_TOL` in `camera_worker.py` |
| Robot spins endlessly in search | Search timeout too long or spiral disabled | Check `search_timeout` and `spiral_search_enabled` in Settings → Search |
| Distance estimate wrong | Camera not calibrated | Re-calibrate: Settings → Sensors → Calibration → place target at 1m → CALIBRATE |
| Similar object detected as target | YOLO model confusion | Retrain the model with more negative examples of the confusing object, or increase `confidence_threshold` |
| Robot jerks during navigation | Kalman filter too responsive | Increase `kf_q_dist` (process noise) in `config.py` for smoother tracking |
| LiDAR distance not matching camera | Sensor mounting offset wrong | Check LiDAR/camera positions in Settings → Sensors → Mounting |
