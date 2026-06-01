# Hardware Setup Guide

> Complete hardware wiring, Arduino protocol, sensor configuration, and physical robot specifications.

---

## Table of Contents

1. [Physical Robot Specifications](#1-physical-robot-specifications)
2. [Hardware Components](#2-hardware-components)
3. [Wiring and Connections](#3-wiring-and-connections)
4. [Arduino Motor Controller Protocol](#4-arduino-motor-controller-protocol)
5. [LiDAR Configuration](#5-lidar-configuration)
6. [Camera Configuration](#6-camera-configuration)
7. [URDF Frame Definitions](#7-urdf-frame-definitions)
8. [Mecanum Wheel Kinematics](#8-mecanum-wheel-kinematics)
9. [Troubleshooting Hardware](#9-troubleshooting-hardware)

---

## 1. Physical Robot Specifications

```
            ┌─────────────────────────┐
            │      USB Camera          │  ← /dev/video0 (front-facing, 640x480)
            │     ┌───────────┐        │
            │     │  RPLidar  │        │  ← /dev/ttyUSB0 (360° 2D laser)
            │     │    A1     │        │
            │     └───────────┘        │
            │                          │
  ┌────┐    │   RZ/V2N Board           │    ┌────┐
  │ FL │────│   + Motor Shield         │────│ FR │
  │    │    │   (PCA9685 I2C)          │    │    │
  └────┘    │                          │    └────┘
            │   Arduino                │
            │   /dev/ttyACM0           │
  ┌────┐    │                          │    ┌────┐
  │ RL │────│   Battery                │────│ RR │
  │    │    │                          │    │    │
  └────┘    └─────────────────────────┘    └────┘
```

### Dimensions

| Parameter | Value | Description |
|-----------|-------|-------------|
| Base size | 300 x 260 x 150 mm | Robot body (L x W x H) |
| Base mass | ~5.0 kg | Approximate total weight |
| Wheel diameter | 80 mm (radius 40 mm) | Mecanum wheels |
| Wheelbase | 190 mm | Front-rear axle distance |
| Track width | 210 mm | Left-right wheel distance |
| LiDAR height | 150 mm above base | Mounted on top |
| Camera offset | 150 mm forward, 100 mm above base | Front-facing |

### Wheel Layout (Top View)

```
        FRONT
   FL ─────── FR
   │           │
   │   ROBOT   │
   │           │
   RL ─────── RR
        REAR

Wheel indices (firmware order):
  FL = index 0
  RL = index 1
  RR = index 2
  FR = index 3
```

---

## 2. Hardware Components

| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Compute board | Renesas RZ/V2N | — | Main computer (Yocto Linux, ROS2 Humble) |
| Motor shield | QGPMaker (PCA9685 I2C) | I2C addr 0x60 | PWM generation for H-bridges |
| H-bridges | TB67H450 (x4) | — | Motor drivers (one per wheel) |
| Motors | DC with encoders (x4) | — | Mecanum wheel drive |
| Encoders | 4320 CPR (x4) | — | Wheel rotation sensing |
| Microcontroller | Arduino (Mega/Uno) | USB Serial | Motor control + encoder reading |
| LiDAR | RPLidar A1 | USB Serial | 360° 2D laser scanner |
| Camera | USB camera (generic) | USB V4L2 | Front-facing for YOLO detection |
| Display | DSI touchscreen | DSI-1 | GUI output (Wayland/Weston, rotated 180°) |
| Network | WiFi AP | WiFi | Access point at 192.168.50.1 |

---

## 3. Wiring and Connections

### USB Ports

| Device | Port | Baud Rate | Protocol |
|--------|------|-----------|----------|
| Arduino | auto-detected (e.g. `/dev/ttyUSB1` or `/dev/ttyACM0`) | 115200 | Plain text commands |
| LiDAR | `/dev/ttyUSB0` (rplidar driver) | 115200 | rplidar protocol |
| Camera | `/dev/video0` | — | V4L2 video |

> The Arduino port is **not fixed**. Whichever number it enumerates as
> (`ttyUSB*` or `ttyACM*`) depends on the USB-serial chip and plug order;
> the driver finds it automatically (see below). The startup/runtime scripts
> are port-agnostic and never hardcode a specific number.

### Arduino Auto-Detection

The Arduino is identified by a **firmware handshake**, not by USB vendor ID.
`ArduinoBridge.find_arduino_port()` ([hardware/arduino_bridge.py](../target_nav/hardware/arduino_bridge.py)) probes candidate serial ports and keeps only the one that **answers the firmware protocol**:

1. **Skip busy ports.** Any port already held open by another process (checked
   via `/proc/*/fd`) is skipped — this is what stops the probe from ever
   opening the lidar's port and disrupting it.
2. **Probe in priority order.** The configured port first, then likely-Arduino
   chips (`VID_TIER1` = Arduino `0x2341`/`0x2A03`, CH340 `0x1A86`), then generic
   serial chips (`VID_TIER2` = CP210x `0x10C4`, FTDI `0x0403`). VIDs only set the
   probe *order* — they never decide identity.
3. **Handshake test.** `_probe_port()` opens the port, sends `READ`, and accepts
   it only if the reply looks like the firmware (`READY`/`OK`/`ODOM`/`ENC`/`DONE`
   or bare encoder integers).

This means a peripheral that merely shares a USB-serial chip (e.g. an RPLidar on
a CP2102) is never mistaken for the Arduino, regardless of port numbering. The
actually-opened device is reported on `/arduino/status` and in the connect log.

### Motor Shield I2C

```
V2N Board ──► Arduino ──► PCA9685 Motor Shield (I2C addr 0x60)
                              ├── TB67H450 H-bridge → FL motor (index 0)
                              ├── TB67H450 H-bridge → RL motor (index 1)
                              ├── TB67H450 H-bridge → RR motor (index 2)
                              └── TB67H450 H-bridge → FR motor (index 3)
```

### Encoder Connections

Each motor has a quadrature encoder (4320 counts per revolution). The Arduino reads all 4 encoders and reports values via serial.

---

## 4. Arduino Motor Controller Protocol

The Arduino firmware is at `/media/shtewe/extSSD/V2N_demoe/rzv2n-arduino-motor-controller/` and uses **PlatformIO** (NOT Arduino IDE).

### Command Format

**Plain text, newline-terminated, NO checksums.**

```
COMMAND[,arg1,arg2,...]\n
```

### Movement Commands

| Command | Format | Description | Requirements |
|---------|--------|-------------|--------------|
| FWD | `FWD,speed,ticks` | Move forward | speed > 0, ticks > 0 |
| BWD | `BWD,speed,ticks` | Move backward | speed > 0, ticks > 0 |
| LEFT | `LEFT,speed,ticks` | Strafe left | speed > 0, ticks > 0 |
| RIGHT | `RIGHT,speed,ticks` | Strafe right | speed > 0, ticks > 0 |
| TURN | `TURN,speed,ticks` | Rotate in place | speed > 0, +ticks=CCW, -ticks=CW |
| DIAGFL | `DIAGFL,speed,ticks` | Diagonal front-left | speed > 0, ticks > 0 |
| DIAGFR | `DIAGFR,speed,ticks` | Diagonal front-right | speed > 0, ticks > 0 |
| DIAGBL | `DIAGBL,speed,ticks` | Diagonal back-left | speed > 0, ticks > 0 |
| DIAGBR | `DIAGBR,speed,ticks` | Diagonal back-right | speed > 0, ticks > 0 |

### Continuous Velocity Command (Best for Navigation)

```
VEL,vx,vy,wz
```

| Parameter | Unit | Description |
|-----------|------|-------------|
| vx | mm/s | Forward/backward velocity |
| vy | mm/s | Left/right strafe velocity |
| wz | mrad/s | Rotation velocity (+ = CCW) |

**200ms Watchdog**: Motors automatically stop if no `VEL` command is received within 200ms. The ROS node must resend at >=5 Hz.

**Dead Zone**: Commands with `|vx|, |vy|, |wz| < 3` trigger automatic STOP. The firmware converts mm/s to motor PWM internally using encoder feedback.

### Simple Commands

| Command | Description |
|---------|-------------|
| `STOP` | Emergency stop all motors |
| `READ` | Read 4 encoder values (returns 4 lines: FL, RL, RR, FR) |
| `SYNC` | Synchronize communication |
| `CALIB` | Start motor calibration (~40 seconds) |
| `RESET` | Reset encoder counters to zero |

### GUI Hardware Tools (via Settings → Tools tab)

The Settings window provides three sub-tabs for hardware testing. All commands are sent via the `/arduino/cmd` ROS2 topic (`std_msgs/String`) directly to the firmware.

**Drive sub-tab** — 8-direction pad + rotation with PWM speed and ticks sliders:

| Button | Firmware Command | Description |
|--------|-----------------|-------------|
| FWD | `FWD,speed,ticks` | Drive forward |
| BWD | `BWD,speed,ticks` | Drive backward |
| LEFT | `LEFT,speed,ticks` | Strafe left |
| RIGHT | `RIGHT,speed,ticks` | Strafe right |
| FL/FR/BL/BR | `DIAGFL,speed,ticks` etc. | Diagonal movement |
| TL | `TURN,speed,ticks` | Rotate CCW |
| TR | `TURN,speed,-ticks` | Rotate CW |
| STOP | `STOP` | Emergency stop |

**Motors sub-tab** — single motor test with raw PWM:

| Button | Firmware Command | Description |
|--------|-----------------|-------------|
| FL/FR/RL/RR | `TMOTOR,id,pwm` | Test one motor at raw PWM |
| STOP | `STOP` | Stop motor |

**System sub-tab** — odometry reset and motor calibration:

| Button | Command | Purpose |
|--------|---------|---------|
| RESET ODOMETRY | (ROS2 /reset_odom) | Reset pose to (0, 0, 0) |
| START CALIBRATION | `CALIB` | Full motor calibration (~40s, 60s countdown timer) |
| ABORT | `STOP` | Abort calibration in progress |

Calibration shows a 60s countdown timer in the GUI. The firmware typically finishes in ~40s (dead-zone + forward + reverse). The 60s is a safe estimate.

This is the recommended way to test and calibrate motors — no terminal access needed.

**Mode switching (VEL vs position):** The ArduinoDriverNode uses a `_vel_mode` flag to prevent the 20Hz VEL command_loop from interfering with GUI position commands. When `/arduino/cmd` receives a command (FWD, TMOTOR, CALIB), `_vel_mode` is set to False and the command_loop goes idle. When `/cmd_vel` receives a velocity (navigation active), `_vel_mode` is set to True and the command_loop resumes sending VEL commands.

### Firmware Responses

| Response | Meaning |
|----------|---------|
| `READY` | Arduino booted and ready |
| `OK` | Command accepted |
| `DONE` | Operation completed |
| `BUSY` | Motors busy with previous command |
| `ERROR: msg` | Error with description |

### Telemetry

```
ODOM,vx_mm,vy_mm,wz_mrad      # Odometry velocities (20Hz, VEL mode only)
ENC,FL:ticks,RL:ticks,RR:ticks,FR:ticks,t_us:microseconds  # Raw encoder ticks (1Hz idle, 20Hz position/test modes)
CALIB,progress                  # During calibration
STALL,motor_info                # Stall detection
```

**Note:** Only ODOM telemetry is consumed by the OdometryNode for navigation. ENC telemetry (sent during IDLE and position/test modes) is ignored by the ROS2 side — the firmware already computes forward kinematics in VEL mode.

### Critical Requirements

- **speed MUST be > 0** for all movement commands (FWD, BWD, etc.)
- **ticks MUST be > 0** for timed movements (ticks=0 causes firmware failure)
- **200ms watchdog** in VEL mode — must resend within 200ms or motors stop
- **No checksums** — plain ASCII text, newline-terminated
- **Baudrate**: 115200
- **Minimum PWM**: 20 (below this, motors stall due to friction)
- **Maximum PWM**: 255

### Connection Sequence

```
Python                          Arduino
  │                                │
  │ ─── Open serial 115200 ──────►│
  │                                │ (Arduino resets on DTR)
  │                    (wait 5s)   │
  │ ◄────────── "READY\n" ────────│
  │                                │
  │ ─── "VEL,200,0,0\n" ────────►│  (200 mm/s forward)
  │ ◄────────── "OK\n" ──────────│
  │ ◄────── "ODOM,..." (20Hz) ───│
  │                                │
  │ ─── "STOP\n" ───────────────►│
  │ ◄────────── "DONE\n" ────────│
```

### Velocity Conversion (Python Side)

```python
# From m/s and rad/s to mm/s and mrad/s:
vx_mm = int(linear_x * 1000)     # m/s -> mm/s
vy_mm = int(linear_y * 1000)     # m/s -> mm/s
wz_mrad = int(angular_z * 1000)  # rad/s -> mrad/s

# Send: VEL,vx_mm,vy_mm,wz_mrad
# Firmware handles mm/s -> PWM conversion internally using encoder feedback.
```

### End-to-End Motor Control Flow (VEL Mode)

This is the complete path from "navigate to target" to "wheels spinning", showing every layer and conversion. This loop repeats ~10 times per second until the robot arrives.

```
Step 1: NAVIGATION DECIDES VELOCITY
────────────────────────────────────
  Nav node (Core 1, 20Hz control loop):
    - Camera/DRP-AI detected a bowling pin 5m ahead at 10° left
    - Reads current robot pose from /odom
    - Computes error: distance=5.0m, angle=10°
    - VFH obstacle avoidance checks LiDAR for obstacles
    - Speed ramp: far from target -> full speed 0.20 m/s
    - Publishes Twist on /cmd_vel:
        linear.x  = 0.20 m/s   (forward)
        linear.y  = 0.00 m/s   (no strafe)
        angular.z = 0.30 rad/s (turn left to align)

Step 2: ROS2 NODE CONVERTS AND SENDS
─────────────────────────────────────
  ArduinoDriverNode (arduino_node.py):
    a. _cmd_vel_callback() stores Twist + timestamps it
       (does NOT send to Arduino here — decouples nav rate from send rate)
    b. 20Hz timer (_command_loop) fires every 50ms:
       - Checks: is last cmd_vel fresh? (< 0.5s old) -> YES
       - Converts ROS2 SI units to firmware integer units:
           0.20 m/s  × 1000 = 200 mm/s
           0.00 m/s  × 1000 =   0 mm/s
           0.30 rad/s × 1000 = 300 mrad/s
       - Calls bridge.send_velocity(200, 0, 300)
    c. ArduinoBridge (arduino_bridge.py):
       - Deadzone check: all values >= 3 -> not zero, proceed
       - Builds ASCII: "VEL,200,0,300\n"
       - Writes to the auto-detected Arduino port (USB serial, 115200 baud)

Step 3: FIRMWARE PARSES COMMAND
───────────────────────────────
  serial_cmd.cpp:
    - update() reads chars one-by-one into 32-byte buffer
    - On '\n': parse() splits on commas
    - Matches "VEL" prefix -> extracts vx=200, vy=0, wz=300
    - Creates Command{type=VELOCITY, vx=200, vy=0, wz=300}

Step 4: STATE MACHINE DISPATCHES
─────────────────────────────────
  robot.cpp handleCommand(VELOCITY):
    a. Mecanum inverse kinematics:
       Mecanum::computeFromVelocity(200, 0, 300, tickRates[])
         - wz_mm = 200mm (lx+ly) × 300 mrad/s / 1000 = 60 mm/s
         - wheel_FL = 200 - 0 - 60 = 140 mm/s
         - wheel_FR = 200 + 0 + 60 = 260 mm/s
         - wheel_RL = 200 + 0 - 60 = 140 mm/s
         - wheel_RR = 200 - 0 + 60 = 260 mm/s
         - Convert mm/s → ticks/period (× 0.3438):
           tickRates = [48, 89, 48, 89] (FL, FR, RL, RR)
    b. Motion::setMotorTickRates(tickRates)
       - Stores as velSetpoint[0..3] for PID
    c. Resets watchdog: lastCommandTime = millis()
       watchdogEnabled = true (200ms timeout active)
    d. State -> MOVING (if not already)

Step 5: PID LOOP DRIVES MOTORS (50Hz, every 20ms)
──────────────────────────────────────────────────
  Timer1 ISR fires -> sets tick flag
  main.cpp: if (Timer::consumeTick()) -> Robot::onTick()
    -> Motion::update()

  For EACH of the 4 motors independently:
    1. READ ENCODER (atomic, interrupts disabled ~16μs):
       currentTicks = Encoder::getSnapshot()
       actualSpeed = |currentTicks - prevTicks|  (ticks since last tick)
       prevTicks = currentTicks

    2. PID COMPUTATION:
       error = |targetRate| - actualSpeed
       integral += error  (clamped to ±150, anti-windup)

       Feed-forward (initial estimate from calibration):
         ff = |targetRate| × 255 / calMaxTickrate[motor]
         (uses calMaxTickrateRev if motor going backward)

       PI correction:
         correction = 1.5 × error + 0.3 × integral

       Final PWM = ff + correction
       Clamped to [0, 255]
       Dead-zone: if PWM > 0 but < calDeadZone[motor], jump to deadzone
       Apply sign from kinematics (+ forward, - backward)

    3. RESULT (example):
       pwm[FL]=90, pwm[FR]=167, pwm[RL]=90, pwm[RR]=167

  Motor::setAll(pwm) -> writes all 4 motors atomically

Step 6: PWM → PHYSICAL MOTOR ROTATION
──────────────────────────────────────
  motor.cpp setAll():
    - For each motor, converts PWM 0-255 to 12-bit duty (× 16):
        FL: PWM +90  -> IN1(CH8)=1440,  IN2(CH9)=0     (forward)
        FR: PWM +167 -> IN1(CH13)=2672, IN2(CH12)=0    (forward)
        RL: PWM +90  -> IN1(CH10)=1440, IN2(CH11)=0    (forward)
        RR: PWM +167 -> IN1(CH15)=2672, IN2(CH14)=0    (forward)
    - flushBuffer() sends all 8 channels in 2 I2C batch transactions

  PCA9685 chip (I2C address 0x60, on motor shield):
    - Generates 1600Hz PWM signal per channel
    - CH8 at 35% duty, CH13 at 65% duty, etc.

  TB67H450 H-Bridge (one per motor):
    - IN1=PWM, IN2=LOW -> Forward mode
    - Switches 12V battery to motor at the duty cycle

  DC Motor + 90:1 Gearbox:
    - Motor shaft spins (~3360 RPM at 35% duty)
    - Gearbox output: ~37 RPM
    - Mecanum wheel turns -> robot moves

  Quadrature Encoder (4320 counts/revolution):
    - Pin-change interrupt fires on each tick
    - Arduino ISR increments counter atomically
    - This feeds back to Step 5 on the next 50Hz tick

Step 7: ODOMETRY FEEDBACK (20Hz, every 50ms)
─────────────────────────────────────────────
  robot.cpp handleMoving():
    - printOdom() reads 4 encoder snapshots (atomic)
    - Computes delta ticks since last ODOM report
    - Mecanum::forwardKinematics(deltas) -> vx_mm, vy_mm, wz_mrad
    - Sends "ODOM,198,0,288\n" over serial

  ArduinoBridge (background read thread on V2N):
    - Reads line, parses: command="ODOM", args=["198","0","288"]
    - Calls on_response callback

  ArduinoDriverNode._handle_arduino_response():
    - Publishes JSON on /arduino/odom_raw topic

  OdometryNode:
    - Converts mm/s back to m/s, mrad/s to rad/s
    - Integrates velocity -> updates robot pose (x, y, theta)
    - Publishes ROS2 Odometry on /odom
    - Broadcasts TF: odom -> base_link

  Nav node (Core 1):
    - Reads /odom -> knows current position
    - Computes new error vs target
    - Publishes updated Twist -> BACK TO STEP 1

Step 8: ROBOT ARRIVES — STOPS
─────────────────────────────
  Nav node: distance_error < 0.22m for 0.3s -> ARRIVED
    -> Stops publishing /cmd_vel

  Safety Layer 1: ArduinoDriverNode (0.5s timeout)
    -> No cmd_vel for 0.5s -> sends "STOP\n"

  Safety Layer 2: Firmware watchdog (200ms)
    -> No VEL for 200ms -> Motor::coastAll()
    -> Sends "ERROR: Watchdog" (or "DONE" if STOP received)

  Safety Layer 3: TB67H450 hardware
    -> IN1=LOW, IN2=LOW -> coast mode -> motor freewheels to stop

  Robot is stopped. Waiting for next /cmd_vel.
```

**Key rates in the flow:**

| Component | Rate | What it does |
|-----------|------|-------------|
| Nav control loop | 20 Hz | Decides velocity from camera + LiDAR + odometry |
| ArduinoDriverNode timer | 20 Hz | Converts m/s → mm/s, sends VEL over serial |
| Firmware PID loop | 50 Hz | Reads encoder, adjusts PWM per motor |
| PCA9685 PWM output | 1600 Hz | Switches motor power on/off |
| Encoder interrupts | ~7680 Hz at full speed | Counts wheel rotation ticks |
| ODOM telemetry | 20 Hz | Reports actual velocity back to ROS2 |
| Firmware watchdog | 200ms timeout | Auto-stops if no VEL received |
| ArduinoDriverNode timeout | 0.5s | Sends STOP if no cmd_vel received |

---

## 5. LiDAR Configuration

### RPLidar A1 Specifications

| Parameter | Value |
|-----------|-------|
| Range | 0.15 – 12.0 m |
| Frequency | ~5.5 Hz (scan rate) |
| Angular resolution | ~1° (360 points/scan) |
| Interface | USB Serial, 115200 baud |
| Port | `/dev/ttyUSB0` |

### ROS2 Driver

The `rplidar_ros` package publishes `/scan` (sensor_msgs/LaserScan).

### Software Filter (in Navigator)

```python
# LiDAR points are filtered:
# - NaN/Inf values → skipped
# - x <= 0 → behind robot, skipped
# - x > slowdown_distance → too far, skipped
# - |y| > robot_half_width + 0.4m → outside corridor, skipped
```

---

## 6. Camera Configuration

### Default Settings

| Parameter | Value |
|-----------|-------|
| Device | `/dev/video0` (device_id=0) |
| Backend | V4L2 (`cv2.CAP_V4L2`) |
| Resolution | 640 x 480 |
| FPS | 30 |
| Buffer size | 1 (minimize latency) |
| Color format | BGR (OpenCV default) |

### Why V4L2 (Not GStreamer)

The V2N board's GStreamer can conflict with DRP-AI hardware acceleration. Using V4L2 directly avoids these issues.

### Camera Thread Behavior

1. Opens camera with retry (3 attempts, 1s between)
2. Captures at ~30fps
3. Submits frames to YOLO detector asynchronously
4. Draws cached detections on display frame
5. Auto-reconnects after 30 consecutive read failures

### Distance Estimation (Pinhole Camera Model)

```
distance = reference_distance * (reference_bbox_height / current_bbox_height)
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `reference_bbox_height` | 180 px | Bounding box height at reference distance |
| `reference_distance` | 1.0 m | Known distance for calibration |
| `horizontal_fov` | 60° | Camera field of view |

### Angle Estimation

```
focal_length = (frame_width / 2) / tan(fov / 2)
offset_x = center_x - frame_width / 2
angle = atan2(offset_x, focal_length)
```

### Calibration Walkthrough

To calibrate distance estimation for your camera:

1. Place a bowling pin at exactly **1.0 meter** from the camera
2. Open the GUI → Settings → Sensors → Calibration
3. Enter **1.0** in the "Known dist" spinner
4. Press **CALIBRATE** — the system reads the bounding box height automatically
5. The result label shows: `OK: 180px @ 1.00m` (your values will vary)
6. The reference values are saved automatically

For manual calibration, adjust the H (height in pixels) and @ (distance in meters) spinners directly.

If the robot consistently under/over-estimates distance, re-calibrate at a different distance (e.g., 0.5m or 2.0m) for better accuracy at that range.

---

## 7. URDF Frame Definitions

The robot's physical dimensions are defined in `urdf/v2n_robot.urdf`.

### Frame Hierarchy

```
base_footprint (ground plane)
  └── base_link (robot center, 0mm above ground)
      ├── laser (LiDAR sensor)
      │   Position: (+120mm, 0, +150mm) — 12cm forward, 15cm above base
      │
      ├── camera_link (front camera)
      │   Position: (+150mm, 0, +100mm) — 15cm forward, 10cm above
      │
      ├── wheel_fl (front-left wheel)
      │   Position: (+95mm, +105mm, 0) — visual only
      │
      ├── wheel_fr (front-right wheel)
      │   Position: (+95mm, -105mm, 0) — visual only
      │
      ├── wheel_rl (rear-left wheel)
      │   Position: (-95mm, +105mm, 0) — visual only
      │
      └── wheel_rr (rear-right wheel)
          Position: (-95mm, -105mm, 0) — visual only
```

### Important Notes

- Wheel frames are **visual only** — no joints for simulation
- The `base_link → laser` transform tells the navigation system where the LiDAR is
- The `base_link → camera_link` transform is used for vision → base_link coordinate conversion
- `robot_state_publisher` broadcasts these as static TF transforms

---

## 8. Mecanum Wheel Kinematics

### Inverse Kinematics (Body Velocity → Wheel Speeds)

```
Given: vx (forward), vy (strafe), wz (rotation)

L = (wheelbase + track_width) / 2 = (0.190 + 0.210) / 2 = 0.200 m

FL = (vx - vy - L * wz) / wheel_radius
FR = (vx + vy + L * wz) / wheel_radius
RL = (vx + vy - L * wz) / wheel_radius
RR = (vx - vy + L * wz) / wheel_radius
```

### Motion Examples

| Motion | vx | vy | wz | FL | FR | RL | RR |
|--------|----|----|----|----|----|----|-----|
| Forward | + | 0 | 0 | + | + | + | + |
| Backward | - | 0 | 0 | - | - | - | - |
| Strafe Left | 0 | + | 0 | - | + | + | - |
| Strafe Right | 0 | - | 0 | + | - | - | + |
| Rotate CCW | 0 | 0 | + | - | + | - | + |
| Rotate CW | 0 | 0 | - | + | - | + | - |
| Diagonal FL | + | + | 0 | 0 | + | + | 0 |
| Diagonal FR | + | - | 0 | + | 0 | 0 | + |

### Forward Kinematics (Wheel Speeds → Body Velocity)

```
vx = (FL + FR + RL + RR) / 4 * wheel_radius
vy = (-FL + FR + RL - RR) / 4 * wheel_radius
wz = (-FL + FR - RL + RR) / (4 * L) * wheel_radius
```

### Speed Limits

| Parameter | Value | Description |
|-----------|-------|-------------|
| Max wheel speed | 10.9 rad/s | At PWM 255 (measured) |
| Max linear speed | 0.436 m/s | Measured at PWM 255 |
| Max angular speed | 2.18 rad/s | Measured at PWM 255 |
| Software linear limit | 0.20 m/s | Default (configurable 0.05–0.60) |
| Software angular limit | 0.40 rad/s | Default (configurable 0.10–1.50) |
| Minimum PWM | 20 | Below this, motors stall |

### Odometry Covariance

| Parameter | Value | Notes |
|-----------|-------|-------|
| Position X | 0.005 m | Forward direction |
| Position Y | 0.010 m | Lateral (more slip) |
| Heading | 0.01 rad | Gyro-less drift |
| Velocity X | 0.01 m/s | |
| Velocity Y | 0.02 m/s | Noisier strafe |
| Angular velocity | 0.02 rad/s | |

---

## 9. Troubleshooting Hardware

### Verify All Devices

```bash
# List all USB-serial devices (Arduino + LiDAR enumerate here; numbers vary).
# Identify which is which by chip: cp210x = LiDAR, ch341/Arduino = motor board.
ls -la /dev/ttyUSB* /dev/ttyACM* /dev/video0
dmesg | grep -iE "cp210|ch34|ttyACM|ttyUSB"

# Check a serial port responds (substitute the real port from above)
stty -F /dev/ttyUSB0 115200

# Check Camera
v4l2-ctl --device=/dev/video0 --all
```

### Arduino Not Responding

```bash
# List USB-serial ports (Arduino is whichever speaks the firmware protocol)
ls /dev/ttyUSB* /dev/ttyACM*

# Check what process is using a port
fuser /dev/ttyUSB* /dev/ttyACM*

# Kill processes holding the ports and retry
fuser -k /dev/ttyUSB* /dev/ttyACM*

# Test manually (substitute the Arduino's real port)
screen /dev/ttyUSB1 115200
# Type: STOP (should get DONE)
# Type: READ (should get 4 encoder values)
# Exit: Ctrl+A, then K
```

### LiDAR Not Spinning

```bash
# Check port
ls /dev/ttyUSB*

# Test scan
ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0
```

### Camera Not Working

```bash
# List cameras
ls /dev/video*

# Test capture
v4l2-ctl --device=/dev/video0 --stream-mmap --stream-count=1

# Check OpenCV
python3 -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened()); cap.release()"
```

### Display Issues (GUI Not Showing)

```bash
# Check Weston
pgrep -x weston

# Check Wayland socket
ls /run/wayland-*

# Set variables manually
export GDK_BACKEND=wayland
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/run
```

### Motor Stalling or Not Moving

1. Check minimum PWM: speed must be > 20 (motor friction threshold)
2. Check ticks: must be > 0 for timed movements
3. Check watchdog: VEL commands must arrive within 200ms
4. Check I2C: `i2cdetect -y 1` should show device at 0x60
5. Check battery voltage: low battery causes stalls

### Permission Issues

```bash
# Add user to dialout group (for serial ports)
usermod -a -G dialout root

# Add user to video group (for camera)
usermod -a -G video root
```
