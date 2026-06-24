"""Single source of truth for every target_nav parameter.

Every configurable value, hardware constant, and default setting lives here.
Other modules import from this file -- never define constants elsewhere.

Persistence:
    User-tunable parameters (navigation, detection, calibration, DRP-AI, map
    display) are persisted to ``~/.config/target_nav/calibration.json`` by
    ``SettingsStore.save_calibration()``.  On startup ``_load_calibration()``
    reads them back and merges with defaults defined here so new keys are
    added transparently.

    DRP-AI C++ binary parameters are additionally written to
    ``/root/deploy/config.ini`` by ``SettingsStore.write_drpai_config_ini()``
    because the C++ binary reads that INI file (not the JSON) on startup.

Cross-references:
    - urdf/v2n_robot.urdf dimensions are for RViz only; these values are authoritative
    - Navigator algorithm constants (VFH, blind approach, arrival scoring)
      are defined in nav/navigator.py lines 42-104.  They are internal tuning
      values not exposed to users -- kept in the module that uses them.
"""

import os

# -- Calibration file path --
CALIBRATION_FILE = os.path.expanduser("~/.config/target_nav/calibration.json")

# ---------------------------------------------------------------------------
# Hardware constants (physical facts, not user-tunable)
# ---------------------------------------------------------------------------
LIDAR_MIN_RANGE = 0.15          # RPLidar A1 minimum range (meters)
LIDAR_MAX_RANGE = 12.0          # RPLidar A1 maximum range (meters)
DEFAULT_WHEEL_RADIUS = 0.04     # 80mm diameter mecanum wheels
DEFAULT_WHEELBASE = 0.190       # front-rear axle distance (meters)
DEFAULT_TRACK_WIDTH = 0.210     # left-right wheel distance (meters)
DEFAULT_ENCODER_CPR = 4320      # encoder counts per wheel revolution
DEFAULT_MAX_PWM = 255           # firmware PWM limit (-255 to +255)
DEFAULT_ARDUINO_PORT = '/dev/ttyACM0'   # Arduino serial device
DEFAULT_LIDAR_PORT = '/dev/ttyUSB0'     # RPLidar serial device
DEFAULT_BAUDRATE = 115200               # serial baud rate (Arduino + LiDAR)
ARDUINO_READY_TIMEOUT = 5.0             # seconds to wait for firmware READY
ARDUINO_MAX_RECONNECT_BACKOFF = 30.0    # max seconds between reconnect attempts

# Maximum robot speeds — used by navigator for speed capping only.
# The firmware accepts VEL commands in mm/s and mrad/s directly and
# handles the conversion to motor PWM internally using encoder feedback.
DEFAULT_MAX_LINEAR_SPEED = 0.436        # m/s (measured at full power)
DEFAULT_MAX_ANGULAR_SPEED = 2.18        # rad/s (measured at full power)

# Shared memory paths (C++ DRP-AI binary ↔ Python, and Python ↔ Python)
SHM_CAMERA_PATH = '/dev/shm/v2n_camera'         # annotated BGR frames
SHM_DETECTIONS_PATH = '/dev/shm/v2n_detections'  # binary detection structs
SHM_CALIBRATION_PATH = '/dev/shm/v2n_calibration' # calibration params
SHM_SETTINGS_PATH = '/dev/shm/v2n_settings'      # JSON settings (cross-process sync)
SHM_SETTINGS_MAX_SIZE = 32768                     # 32 KB — more than enough for settings JSON

# ---------------------------------------------------------------------------
# Robot body dimensions (meters) -- from URDF v2n_robot.urdf
# ---------------------------------------------------------------------------
DEFAULT_ROBOT = {
    'length': 0.30,       # front-to-back (X)
    'width': 0.26,        # left-to-right (Y)
    'height': 0.15,       # floor-to-top (Z)
}

# ---------------------------------------------------------------------------
# Sensor mounting positions (meters from base_link center)
# ---------------------------------------------------------------------------
DEFAULT_LIDAR = {'x': 0.12, 'y': 0.0, 'z': 0.15}
DEFAULT_LIDAR_YAW = 180.0   # LiDAR mounting angle (degrees)

DEFAULT_CAMERA = {'x': 0.15, 'y': 0.0, 'z': 0.10}
DEFAULT_CAMERA_YAW = 0.0    # camera mounting angle (degrees) relative to robot forward
DEFAULT_CAMERA_FLIP_H = False  # horizontal flip — set True if camera mounted mirrored

# ---------------------------------------------------------------------------
# Camera optics defaults
# ---------------------------------------------------------------------------
DEFAULT_CAMERA_FOV = 60.0    # horizontal FOV degrees
DEFAULT_FRAME_W = 640        # frame width in pixels
DEFAULT_FRAME_H = 480        # frame height in pixels

# ---------------------------------------------------------------------------
# Vision distance calibration defaults
# ---------------------------------------------------------------------------
DEFAULT_REF_BOX_HEIGHT = 180.0  # bounding box height in pixels at reference distance
DEFAULT_REF_DISTANCE = 1.0      # reference distance in meters

# ---------------------------------------------------------------------------
# Distance reference point
# ---------------------------------------------------------------------------
REF_POINTS = ['center', 'front', 'camera', 'lidar']
DEFAULT_REF_POINT = 'front'

# ---------------------------------------------------------------------------
# Target real-world dimensions (meters)
# ---------------------------------------------------------------------------
DEFAULT_TARGET = {
    'height': 0.28,       # bowling pin height (meters)
    'width': 0.08,        # bowling pin width at widest (meters)
}

# ---------------------------------------------------------------------------
# Radar display parameters (GUI only, no effect on navigation)
# ---------------------------------------------------------------------------
DEFAULT_MAP_PARAMS = {
    'radar_range': 5,         # radar view radius in meters (1-10)
    'laser_point_size': 2,    # LiDAR point size in pixels (1-5)
    'robot_arrow_len': 20,    # heading arrow length in pixels
    'show_grid': True,        # show distance circles
    'show_nav_target': True,  # show navigation target diamond
}

# ---------------------------------------------------------------------------
# Target class profile (detection class name, display label, accepted classes)
# ---------------------------------------------------------------------------
TARGET_CLASS_NAME = "bowling-pin"
TARGET_DISPLAY_NAME = "Target"
TARGET_FILTER_CLASSES = ["bowling-pin"]

# ---------------------------------------------------------------------------
# Detection filter parameters (persisted via save_calibration)
#
# Active parameters (used by the Python detection pipeline at runtime):
#   - confidence_threshold: minimum detection confidence (0.05-0.95)
#   - detect_expiry: seconds before detection is considered stale
#
# Deprecated parameters (persisted for calibration.json backward compatibility
# but filtering is now handled by the C++ DRP-AI binary):
#   - detect_interval, filter_max_aspect, filter_max_box_pct,
#     filter_bbox_window, filter_bbox_factor
# ---------------------------------------------------------------------------
DEFAULT_DETECT_PARAMS = {
    # Active -- used by the Python detection pipeline
    'confidence_threshold': 0.30,   # minimum detection confidence (0.05-0.95)
    'detect_expiry': 4.0,          # seconds before detection is considered stale
    # Below: persisted for calibration.json compatibility but filtering
    # is now handled by the C++ DRP-AI binary. These values are NOT
    # read by the Python pipeline at runtime.
    'detect_interval': 0.0,        # minimum seconds between inferences (0=unlimited)
    'filter_max_aspect': 1.5,      # max width/height ratio (targets are tall, not wide)
    'filter_max_box_pct': 0.80,    # max box area as fraction of frame (reject huge boxes)
    'filter_bbox_window': 5,       # rolling median window for bbox stability (frames)
    'filter_bbox_factor': 3.0,     # bbox outlier rejection factor (higher=more lenient)
}

# ---------------------------------------------------------------------------
# Navigation parameters (persisted via save_calibration)
#
# All speeds in m/s, distances in meters, timeouts in seconds.
#
# How the robot moves:
#
#   FAR (> slowdown_dist)          → drives at linear_speed
#   NEAR (stop_dist..slowdown_dist) → ramps speed down to min_linear_speed
#   STOP (< stop_dist)             → stops (obstacle too close)
#   ARRIVED (< approach_dist)      → target reached, stop
#
#   Target lost for > lost_timeout  → start 360° search scan
#   Search scan > search_timeout    → start spiral search
#   Target very close but lost      → blind approach (dead reckon)
#
# ---------------------------------------------------------------------------
DEFAULT_NAV_PARAMS = {
    # -- Speeds (tuned for 15 FPS detection at ~65ms per frame) --
    # At 0.20 m/s the robot moves ~1.3cm between frames — detection stays locked.
    # At 0.40 m/s the robot moves ~2.6cm — still safe for tracking.
    # Above 0.50 m/s the target can leave the frame between detections.
    'linear_speed': 0.20,                 # max forward speed (m/s)
    'min_linear_speed': 0.10,             # creep speed near obstacles (m/s)
    'angular_speed': 0.40,                # max turning speed (rad/s)

    # -- When to stop --
    'approach_distance': 0.22,            # ARRIVED when this close to target (m)
    'obstacle_distance': 0.22,            # STOP — obstacle closer than this (m)
    'obstacle_slowdown_distance': 0.45,   # START SLOWING — ramp speed down (m)
    #                                       (must be > obstacle_distance)

    # -- Target lost → search --
    'lost_timeout': 3.0,                  # seconds without target → start search
    'search_timeout': 30.0,              # seconds in 360° scan → start spiral
    'search_angular_speed': 0.50,         # rotation speed during 360° search (rad/s)

    # -- Blind approach (target lost at close range) --
    'blind_approach_entry_distance': 0.35, # switch to blind if target lost within this (m)
    'blind_approach_speed': 0.10,          # forward speed while blind (m/s)
    'blind_approach_timeout': 10.0,        # give up blind after this (s)
    'blind_approach_lidar_stop': 0.15,     # LiDAR min range — stop if closer (m)
    'blind_approach_arrival_margin': 0.10,  # close enough in map frame → arrived (m)

    # -- Spiral search (after 360° scan fails) --
    'spiral_search_enabled': True,
    'spiral_initial_radius': 0.3,          # start spiral at this radius (m)
    'spiral_max_radius': 2.0,             # max spiral expansion (m)
    'spiral_growth_rate': 0.15,            # radius grows this much per loop (m)
    'spiral_linear_speed': 0.10,           # forward speed during spiral (m/s)
    'spiral_angular_speed': 0.25,          # turn speed during spiral (rad/s)
    'spiral_timeout': 45.0,              # give up spiral after this (s)
}

# ---------------------------------------------------------------------------
# Target tracker parameters (Kalman filter + validation gates)
#
# Controls the TargetTracker's size-distance gate, LiDAR matching, and
# Kalman filter noise. These are algorithm internals — not user-configurable
# via the Settings GUI (unlike nav params above).
# ---------------------------------------------------------------------------
DEFAULT_TRACKER_PARAMS = {
    # Kalman process noise (state uncertainty growth per second)
    'kf_q_dist': 0.01,            # distance process noise (m²/s)
    'kf_q_angle': 0.005,          # angle process noise (rad²/s)
    'kf_q_ddot': 0.1,             # distance-rate process noise
    'kf_q_adot': 0.05,            # angle-rate process noise

    # Kalman measurement noise
    'kf_r_cam_dist': 0.04,        # camera distance variance (m²) — ~0.2m std dev
    'kf_r_cam_angle': 0.003,      # camera angle variance (rad²) — ~3° std dev
    'kf_r_lidar_dist': 0.001,     # LiDAR distance variance (m²) — ~3cm std dev

    # Size-distance validation gate
    'size_gate_tolerance': 0.5,    # 50% — reject if bbox ratio outside [0.67, 1.5]

    # LiDAR matching window
    'lidar_angle_window': 10.0,    # degrees — angular match window
    'lidar_dist_tolerance': 0.5,   # 50% — distance match tolerance

    # Prediction timeout (coast without detection)
    'predict_timeout': 3.0,        # seconds before track is dropped (must match persist_time above)
}

# ---------------------------------------------------------------------------
# DRP-AI C++ binary parameters (written to deploy/config.ini on the V2N)
#
# The C++ binary reads config.ini once at startup.  To apply changes the
# Python side writes the updated INI and restarts the subprocess.  These
# defaults match the production deploy/config.ini shipped with the robot.
#
# Frequency reference (from Renesas DRP-AI docs):
#   DRP core:  2=420MHz  3=315MHz  4=252MHz  5=210MHz  127=9.8MHz(min)
#   AI-MAC:    1=1GHz    2=1GHz    3=630MHz  4=420MHz  5=315MHz  127=10MHz(min)
# ---------------------------------------------------------------------------
DEFAULT_DRPAI_PARAMS = {
    'conf_threshold': 0.20,      # C++ pre-filter (low — Python does smart filtering)
    'nms_threshold': 0.45,       # C++ NMS overlap threshold (soft-NMS in binary)
    'max_detections': 1,         # single target — avoids multiple bboxes on display
    'drp_max_freq': 2,           # DRP core: 420MHz (fastest)
    'drpai_freq': 5,             # AI-MAC: 315MHz (proven stable)
}

# Path to the DRP-AI config.ini that the C++ binary reads on startup.
# Only meaningful on the V2N robot itself (where the binary runs).
DRPAI_CONFIG_INI_PATH = '/root/deploy/config.ini'

# Human-readable labels for DRP-AI frequency dropdowns in the GUI
DRPAI_DRP_FREQ_OPTIONS = {2: '420 MHz', 3: '315 MHz', 4: '252 MHz', 5: '210 MHz'}
DRPAI_AIMAC_FREQ_OPTIONS = {1: '1 GHz', 3: '630 MHz', 4: '420 MHz', 5: '315 MHz'}

# ---------------------------------------------------------------------------
# Detection tracker constants (temporal stabilization in detection_filters.py)
#
# Controls DetectionTracker -- the lightweight frame-to-frame tracker that
# smooths jittery DRP-AI detections before they reach the navigator.
# These are algorithm internals, not user-tunable via GUI.
# ---------------------------------------------------------------------------
DEFAULT_TRACKER_FILTER = {
    'min_hits': 1,               # not used for gating (all matches output immediately)
    'max_age': 45,               # frames to keep unmatched track (~3s at 15fps)
    'match_dist_px': 150,        # bbox center match distance (pixels, generous for jitter)
    'bbox_ema_alpha': 0.3,       # smoothing factor for bbox display (lower = smoother)
    'persist_time': 3.0,         # seconds to keep outputting after last detection (match predict_timeout)
}

# ---------------------------------------------------------------------------
# Parameter validation ranges (min, max) for every user-tunable value.
# SettingsStore validates against these on EVERY load — bad values in
# the persistence file can never break the system.  If a value is out
# of range, the default from config.py is used silently.
# ---------------------------------------------------------------------------
PARAM_RANGES = {
    # Calibration
    'ref_box_height':       (10.0, 800.0),
    'ref_distance':         (0.05, 4.0),
    'camera_fov':           (10.0, 170.0),
    # Detection
    'confidence_threshold': (0.05, 0.95),
    'detect_expiry':        (0.3, 10.0),
    'detect_interval':      (0.0, 5.0),
    'filter_max_aspect':    (0.5, 5.0),
    'filter_max_box_pct':   (0.1, 1.0),
    'filter_bbox_window':   (1, 50),
    'filter_bbox_factor':   (1.0, 10.0),
    # Navigation
    'approach_distance':    (0.0, 2.0),
    'linear_speed':         (0.05, 0.60),   # safe max ~0.50 at 15 FPS detection
    'min_linear_speed':     (0.03, 0.30),
    'angular_speed':        (0.10, 1.50),
    'obstacle_distance':    (0.05, 2.0),
    'obstacle_slowdown_distance': (0.1, 3.0),
    'search_angular_speed': (0.05, 2.0),
    'lost_timeout':         (0.5, 30.0),
    'search_timeout':       (5.0, 120.0),
    'blind_approach_entry_distance': (0.1, 2.0),
    'blind_approach_speed': (0.01, 0.5),
    'blind_approach_timeout': (1.0, 30.0),
    'blind_approach_lidar_stop': (0.05, 1.0),
    'blind_approach_arrival_margin': (0.01, 1.0),
    'spiral_initial_radius': (0.1, 5.0),
    'spiral_max_radius':    (0.5, 10.0),
    'spiral_growth_rate':   (0.01, 1.0),
    'spiral_linear_speed':  (0.01, 1.0),
    'spiral_angular_speed': (0.01, 2.0),
    'spiral_timeout':       (5.0, 120.0),
    # Robot geometry
    'robot_length':         (0.05, 1.0),
    'robot_width':          (0.05, 1.0),
    'robot_height':         (0.01, 0.5),
    # Sensor mounting positions (meters from base_link center)
    'lidar_x':              (-0.5, 0.5),
    'lidar_y':              (-0.5, 0.5),
    'lidar_z':              (0.0, 0.5),
    'camera_x':             (-0.5, 0.5),
    'camera_y':             (-0.5, 0.5),
    'camera_z':             (0.0, 0.5),
    'lidar_yaw':            (0.0, 360.0),
    'camera_yaw':           (-180.0, 180.0),
    # DRP-AI
    'drpai_conf_threshold': (0.10, 0.95),
    'drpai_nms_threshold':  (0.10, 0.95),
    'drpai_max_detections': (1, 20),
    'drpai_drp_max_freq':   (2, 127),
    'drpai_drpai_freq':     (1, 127),
}


def validate_param(key, value, default):
    """Validate a parameter against PARAM_RANGES, return default if invalid.

    Args:
        key: Parameter name (must exist in PARAM_RANGES).
        value: Value to validate.
        default: Default to use if validation fails.

    Returns:
        The value if valid, otherwise the default.
    """
    if key not in PARAM_RANGES:
        return value  # no range defined, accept as-is
    lo, hi = PARAM_RANGES[key]
    try:
        v = type(default)(value)
        if lo <= v <= hi:
            return v
    except (TypeError, ValueError):
        pass
    return default


# ---------------------------------------------------------------------------
# Thread-safety and timing constants (used across multiple modules)
# ---------------------------------------------------------------------------
STORE_LOCK_TIMEOUT = 0.1         # seconds -- RLock timeout in all state stores
GUI_FRAME_REFRESH_INTERVAL = 0.1 # seconds -- GTK frame refresh timer (10 Hz)

# ---------------------------------------------------------------------------
# Camera worker constants (retry and failure thresholds)
# --------------------------------------------------------------------------
CAMERA_OPEN_RETRIES = 3          # max attempts to open camera device
CAMERA_MAX_READ_FAILURES = 30    # con-secutive read failures before giving up
DETECTION_PIPE_MAX_MISSES = 5    # consecutive empty pipe reads before restart
