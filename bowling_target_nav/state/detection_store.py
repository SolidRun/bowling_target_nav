"""Thread-safe storage for camera frames, detections, and tunable parameters.

All settings are persisted to ~/.config/bowling_target_nav/calibration.json
and synced across processes via mp.Event notifications.

Calibration Workflow
--------------------
1. **Distance calibration** (GUI Settings > Calibration tab):
   - Place the target (bottle) at exactly 1.0m from the camera
   - Click "Calibrate" — the system records the bounding box height in pixels
   - Or manually set reference_box_height and reference_distance

2. **Sensor mounting** (GUI Settings > Sensors tab):
   - Set camera position (x, y, z) relative to base_link center
   - Set camera yaw (degrees) if camera is not pointing forward
   - Set LiDAR position and yaw similarly
   - These affect TF2 transforms and distance fusion

3. **Navigation tuning** (GUI Settings > Navigation tab):
   - approach_distance: How close to stop from target (meters)
   - linear_speed / angular_speed: Max navigation speeds
   - obstacle_distance: Minimum clearance before avoidance
   - blind_approach_*: Parameters for final close-range approach

4. **Map display** (GUI Settings > Map tab):
   - rotation: Rotate map display (degrees)
   - show_grid, show_laser, show_nav_target: Toggle overlays

Settings File Format (calibration.json)
----------------------------------------
    {
        "ref_box_height": 180.0,
        "ref_distance": 1.0,
        "camera_fov": 60.0,
        "camera": {"x": 0.12, "y": 0.0, "z": 0.10},
        "camera_yaw": 0.0,
        "lidar": {"x": 0.0, "y": 0.0, "z": 0.15},
        "lidar_yaw": 0.0,
        "robot": {"length": 0.25, "width": 0.26, "height": 0.15},
        "target": {"height": 0.25, "width": 0.07},
        "ref_point": "front",
        "nav_params": {
            "approach_distance": 0.06,
            "linear_speed": 0.10,
            "obstacle_distance": 0.20,
            ...
        },
        "map_params": {
            "rotation": 0.0,
            "show_grid": true,
            ...
        }
    }
"""

import json
import math
import os
import threading
import time

CALIBRATION_FILE = os.path.expanduser("~/.config/bowling_target_nav/calibration.json")

# Default robot dimensions (meters) - from URDF v2n_robot.urdf
DEFAULT_ROBOT = {
    'length': 0.25,       # front-to-back (X)
    'width': 0.26,        # left-to-right (Y)
    'height': 0.15,       # floor-to-top (Z)
}

# Default sensor positions (meters from base_link center)
DEFAULT_LIDAR = {'x': 0.0, 'y': 0.0, 'z': 0.15}
DEFAULT_LIDAR_YAW = 0.0   # LiDAR mounting angle (degrees) — affects SLAM map orientation
DEFAULT_CAMERA = {'x': 0.12, 'y': 0.0, 'z': 0.10}
DEFAULT_CAMERA_YAW = 0.0  # camera mounting angle (degrees) relative to robot forward

# Default camera optics
DEFAULT_CAMERA_FOV = 60.0    # horizontal FOV degrees
DEFAULT_FRAME_W = 640
DEFAULT_FRAME_H = 480

# Distance reference point options
REF_POINTS = ['center', 'front', 'camera', 'lidar']
DEFAULT_REF_POINT = 'front'

# Default target (bottle) dimensions (meters)
DEFAULT_TARGET = {
    'height': 0.25,       # real object height
    'width': 0.07,        # real object width (diameter)
}

# Default map display parameters
DEFAULT_MAP_PARAMS = {
    'rotation': 0.0,          # map rotation in degrees
    'robot_size': 12,         # robot marker radius in pixels
    'robot_arrow_len': 20,    # heading arrow length in pixels
    'show_grid': True,
    'show_laser': True,
    'show_nav_target': True,
    'laser_point_size': 1,    # 0=1px, 1=3px, 2=5px
}

# Default navigation parameters (persisted via save_calibration)
DEFAULT_NAV_PARAMS = {
    'approach_distance': 0.10,            # arrive when ≤10cm from target
    'linear_speed': 0.10,
    'min_linear_speed': 0.07,
    'angular_speed': 0.3,
    'obstacle_distance': 0.22,            # >= LiDAR min 0.15m with margin
    'obstacle_slowdown_distance': 0.40,
    'search_angular_speed': 0.35,
    'lost_timeout': 3.0,
    'search_timeout': 30.0,
    'blind_approach_entry_distance': 0.37,
    'blind_approach_speed': 0.12,
    'blind_approach_timeout': 10.0,
    'blind_approach_lidar_stop': 0.15,    # = LiDAR min range (clamped in navigator)
    'blind_approach_arrival_margin': 0.10, # reasonable margin for map-frame check
    'spiral_search_enabled': True,
    'spiral_initial_radius': 0.3,
    'spiral_max_radius': 2.0,
    'spiral_growth_rate': 0.15,
    'spiral_linear_speed': 0.10,
    'spiral_angular_speed': 0.25,
    'spiral_timeout': 45.0,
}


class DetectionStore:
    """Thread-safe container for camera/detection data and tunable params.

    All public get/set methods acquire an RLock with a short timeout
    (LOCK_TIMEOUT = 0.1s) so the GUI never blocks indefinitely.
    On lock timeout, getters return safe defaults.
    """

    LOCK_TIMEOUT = 0.1

    def __init__(self):
        self._lock = threading.RLock()
        self._camera_frame = None
        self._detections = []
        self._detection_info = "Initializing..."
        self._detection_time = 0.0

        # Tunable params (accessed by camera thread, modified by settings)
        self._detect_interval = 0.0
        self._detect_expiry = 1.0         # 1s — generous for DRP-AI gaps
        self._confidence_threshold = 0.30  # DRP-AI bottles often 0.3-0.6
        self._ref_box_height = 180.0
        self._ref_distance = 1.0
        self._calibration_dirty = False
        self._stream_mode = False
        self._detector_mode = "Initializing"

        # Detection filter parameters
        self._filter_max_aspect = 1.5     # max bw/bh — bottle is tall, allow some margin
        self._filter_max_box_pct = 0.80   # allow up to 80% of frame (close bottle)
        self._filter_size_tolerance = 2.0 # unused (kept for calibration compat)
        self._filter_tracker_hits = 2     # unused (kept for calibration compat)
        self._filter_tracker_dist = 80    # unused (kept for calibration compat)
        self._filter_bbox_window = 5     # rolling median window size
        self._filter_bbox_factor = 3.0   # generous — reject only wildly wrong sizes
        self._bbox_reset_requested = False

        # Robot dimensions
        self._robot = dict(DEFAULT_ROBOT)

        # Sensor positions
        self._lidar = dict(DEFAULT_LIDAR)
        self._lidar_yaw = DEFAULT_LIDAR_YAW
        self._camera = dict(DEFAULT_CAMERA)

        # Camera optics
        self._camera_yaw = DEFAULT_CAMERA_YAW
        self._camera_fov = DEFAULT_CAMERA_FOV
        self._frame_w = DEFAULT_FRAME_W
        self._frame_h = DEFAULT_FRAME_H

        # Target dimensions
        self._target = dict(DEFAULT_TARGET)

        # Distance reference point
        self._ref_point = DEFAULT_REF_POINT

        # Map display parameters
        self._map_params = dict(DEFAULT_MAP_PARAMS)

        # Navigation parameters
        self._nav_params = dict(DEFAULT_NAV_PARAMS)

        # Load saved calibration from disk
        self._load_calibration()

    def _try_lock(self, timeout=None):
        return self._lock.acquire(timeout=timeout or self.LOCK_TIMEOUT)

    def _release(self):
        try:
            self._lock.release()
        except RuntimeError:
            pass

    # -- Camera frame + detections --
    def set_camera(self, frame, detections, info, fresh_detection=False):
        """Store the latest camera frame and detection results.

        Args:
            frame: RGB numpy array (H, W, 3) or None.
            detections: List of detection dicts.
            info: Human-readable status string (e.g. 'DRP-AI Stream 15fps').
            fresh_detection: If True, update the detection timestamp.

        Returns:
            True on success, False if the lock timed out.
        """
        if not self._try_lock():
            return False
        try:
            self._camera_frame = frame
            self._detections = detections
            self._detection_info = info
            if fresh_detection:
                self._detection_time = time.time()
            return True
        finally:
            self._release()

    def get_camera(self):
        """Return (frame, detections, info_str, det_age_seconds).

        Returns copies to avoid cross-thread mutation.  On lock timeout
        returns (None, [], "Lock timeout", 0.0).
        """
        if not self._try_lock():
            return None, [], "Lock timeout", 0.0
        try:
            frame = self._camera_frame.copy() if self._camera_frame is not None else None
            det_age = time.time() - self._detection_time if self._detection_time > 0 else 999.0
            return frame, self._detections[:], self._detection_info, det_age
        finally:
            self._release()

    # -- Tunable parameters --
    def set_detect_interval(self, val):
        if self._try_lock():
            try: self._detect_interval = val
            finally: self._release()

    def get_detect_interval(self):
        if self._try_lock():
            try: return self._detect_interval
            finally: self._release()
        return 0.0

    def set_detect_expiry(self, val):
        if self._try_lock():
            try: self._detect_expiry = max(0.3, float(val))
            finally: self._release()

    def get_detect_expiry(self):
        if self._try_lock():
            try: return self._detect_expiry
            finally: self._release()
        return 1.0

    def set_confidence_threshold(self, val):
        if self._try_lock():
            try: self._confidence_threshold = max(0.05, min(0.95, float(val)))
            finally: self._release()

    def get_confidence_threshold(self):
        if self._try_lock():
            try: return self._confidence_threshold
            finally: self._release()
        return 0.30

    # -- Detection filter parameters --
    def set_filter_max_aspect(self, val):
        if self._try_lock():
            try: self._filter_max_aspect = max(0.5, min(3.0, float(val)))
            finally: self._release()

    def get_filter_max_aspect(self):
        if self._try_lock():
            try: return self._filter_max_aspect
            finally: self._release()
        return 1.5

    def set_filter_max_box_pct(self, val):
        if self._try_lock():
            try: self._filter_max_box_pct = max(0.4, min(1.0, float(val)))
            finally: self._release()

    def get_filter_max_box_pct(self):
        if self._try_lock():
            try: return self._filter_max_box_pct
            finally: self._release()
        return 0.80

    def set_filter_size_tolerance(self, val):
        if self._try_lock():
            try: self._filter_size_tolerance = val
            finally: self._release()

    def get_filter_size_tolerance(self):
        if self._try_lock():
            try: return self._filter_size_tolerance
            finally: self._release()
        return 2.0

    def set_filter_tracker_hits(self, val):
        if self._try_lock():
            try: self._filter_tracker_hits = int(val)
            finally: self._release()

    def get_filter_tracker_hits(self):
        if self._try_lock():
            try: return self._filter_tracker_hits
            finally: self._release()
        return 2

    def set_filter_tracker_dist(self, val):
        if self._try_lock():
            try: self._filter_tracker_dist = int(val)
            finally: self._release()

    def get_filter_tracker_dist(self):
        if self._try_lock():
            try: return self._filter_tracker_dist
            finally: self._release()
        return 80

    # -- Bbox size tracker params --
    def set_filter_bbox_window(self, val):
        if self._try_lock():
            try: self._filter_bbox_window = max(3, min(50, int(val)))
            finally: self._release()

    def get_filter_bbox_window(self):
        if self._try_lock():
            try: return self._filter_bbox_window
            finally: self._release()
        return 5

    def set_filter_bbox_factor(self, val):
        if self._try_lock():
            try: self._filter_bbox_factor = max(1.2, min(10.0, float(val)))
            finally: self._release()

    def get_filter_bbox_factor(self):
        if self._try_lock():
            try: return self._filter_bbox_factor
            finally: self._release()
        return 2.5

    def request_bbox_reset(self):
        """Signal the camera thread to reset the bbox size tracker."""
        if self._try_lock():
            try: self._bbox_reset_requested = True
            finally: self._release()

    def check_and_clear_bbox_reset(self):
        """Check and clear the bbox reset flag (called by camera thread)."""
        if self._try_lock():
            try:
                val = self._bbox_reset_requested
                self._bbox_reset_requested = False
                return val
            finally: self._release()
        return False

    # -- Distance calibration --
    def set_calibration(self, box_height, distance):
        """Set the reference calibration point for vision distance estimation.

        Args:
            box_height: Bounding box height in pixels at the reference distance.
            distance: Reference distance in meters.
        """
        if self._try_lock():
            try:
                self._ref_box_height = box_height
                self._ref_distance = distance
                self._calibration_dirty = True
            finally: self._release()

    def get_calibration(self):
        """Return (ref_box_height, ref_distance) calibration pair."""
        if self._try_lock():
            try: return self._ref_box_height, self._ref_distance
            finally: self._release()
        return 180.0, 1.0

    def is_calibration_dirty(self):
        """Check if calibration was updated since last check."""
        if self._try_lock():
            try:
                dirty = self._calibration_dirty
                self._calibration_dirty = False
                return dirty
            finally: self._release()
        return False

    # -- Robot dimensions --
    def set_robot_dims(self, length, width, height):
        if self._try_lock():
            try:
                self._robot = {'length': length, 'width': width, 'height': height}
            finally: self._release()

    def get_robot_dims(self):
        if self._try_lock():
            try: return dict(self._robot)
            finally: self._release()
        return dict(DEFAULT_ROBOT)

    def get_robot_half_width(self):
        """Convenience: returns width / 2 for obstacle avoidance."""
        if self._try_lock():
            try: return self._robot['width'] / 2.0
            finally: self._release()
        return DEFAULT_ROBOT['width'] / 2.0

    # -- Sensor positions --
    def set_lidar_pos(self, x, y, z):
        if self._try_lock():
            try: self._lidar = {'x': x, 'y': y, 'z': z}
            finally: self._release()

    def get_lidar_pos(self):
        if self._try_lock():
            try: return dict(self._lidar)
            finally: self._release()
        return dict(DEFAULT_LIDAR)

    # -- LiDAR mounting angle --
    def set_lidar_yaw(self, yaw_deg):
        if self._try_lock():
            try: self._lidar_yaw = yaw_deg
            finally: self._release()

    def get_lidar_yaw(self):
        if self._try_lock():
            try: return self._lidar_yaw
            finally: self._release()
        return DEFAULT_LIDAR_YAW

    def set_camera_pos(self, x, y, z):
        if self._try_lock():
            try: self._camera = {'x': x, 'y': y, 'z': z}
            finally: self._release()

    def get_camera_pos(self):
        if self._try_lock():
            try: return dict(self._camera)
            finally: self._release()
        return dict(DEFAULT_CAMERA)

    # -- Camera mounting angle --
    def set_camera_yaw(self, yaw_deg):
        if self._try_lock():
            try: self._camera_yaw = yaw_deg
            finally: self._release()

    def get_camera_yaw(self):
        if self._try_lock():
            try: return self._camera_yaw
            finally: self._release()
        return DEFAULT_CAMERA_YAW

    # -- Camera optics --
    def set_camera_fov(self, fov_deg):
        if self._try_lock():
            try:
                self._camera_fov = fov_deg
                self._calibration_dirty = True
            finally: self._release()

    def get_camera_fov(self):
        if self._try_lock():
            try: return self._camera_fov
            finally: self._release()
        return DEFAULT_CAMERA_FOV

    # -- Target dimensions --
    def set_target_dims(self, height, width):
        if self._try_lock():
            try:
                self._target = {'height': height, 'width': width}
                self._calibration_dirty = True
            finally: self._release()

    def get_target_dims(self):
        if self._try_lock():
            try: return dict(self._target)
            finally: self._release()
        return dict(DEFAULT_TARGET)

    # -- Distance reference point --
    def set_ref_point(self, point):
        if point in REF_POINTS and self._try_lock():
            try: self._ref_point = point
            finally: self._release()

    def get_ref_point(self):
        if self._try_lock():
            try: return self._ref_point
            finally: self._release()
        return DEFAULT_REF_POINT

    def get_sensor_offset(self, source):
        """Get X offset (meters) to convert a sensor distance to the active reference point.

        Usage: distance_from_ref = sensor_distance + get_sensor_offset(source)

        Args:
            source: 'camera' or 'lidar' — which sensor measured the distance.

        Returns:
            Offset in meters. Positive means the reference is behind the sensor
            (sensor is closer to target), so add to get reference distance.
        """
        if self._try_lock():
            try:
                cam_x = self._camera['x']
                lidar_x = self._lidar['x']
                robot_len = self._robot['length']
                ref = self._ref_point
            finally: self._release()
        else:
            cam_x = DEFAULT_CAMERA['x']
            lidar_x = DEFAULT_LIDAR['x']
            robot_len = DEFAULT_ROBOT['length']
            ref = DEFAULT_REF_POINT

        # Reference point X position (from base_link center)
        if ref == 'center':
            ref_x = 0.0
        elif ref == 'front':
            ref_x = robot_len / 2.0
        elif ref == 'camera':
            ref_x = cam_x
        elif ref == 'lidar':
            ref_x = lidar_x
        else:
            ref_x = 0.0

        # Sensor X position
        sensor_x = cam_x if source == 'camera' else lidar_x

        # Offset = how far reference is behind sensor
        # If sensor is at 0.12m and ref is at 0.125m (front), offset = 0.005
        # distance_from_ref = distance_from_sensor + (ref_x - sensor_x)
        return ref_x - sensor_x

    # -- Map display parameters --
    def set_map_param(self, key, value):
        if key in DEFAULT_MAP_PARAMS and self._try_lock():
            try:
                old = self._map_params.get(key)
                self._map_params[key] = value
                if old != value:
                    print(f"[Store] set_map_param({key}): {old} -> {value}  id={id(self._map_params)}", flush=True)
            finally: self._release()
        elif key not in DEFAULT_MAP_PARAMS:
            print(f"[Store] WARNING: set_map_param({key}) - key not in defaults!", flush=True)

    def get_map_param(self, key):
        if self._try_lock():
            try: return self._map_params.get(key, DEFAULT_MAP_PARAMS.get(key))
            finally: self._release()
        return DEFAULT_MAP_PARAMS.get(key)

    def get_map_params(self):
        if self._try_lock():
            try: return dict(self._map_params)
            finally: self._release()
        print("[Store] WARNING: get_map_params lock FAILED, returning defaults!", flush=True)
        return dict(DEFAULT_MAP_PARAMS)

    # -- Navigation parameters --
    def set_nav_param(self, key, value):
        if key in DEFAULT_NAV_PARAMS and self._try_lock():
            try: self._nav_params[key] = value
            finally: self._release()

    def get_nav_param(self, key):
        if self._try_lock():
            try: return self._nav_params.get(key, DEFAULT_NAV_PARAMS.get(key))
            finally: self._release()
        return DEFAULT_NAV_PARAMS.get(key)

    def get_nav_params(self):
        if self._try_lock():
            try: return dict(self._nav_params)
            finally: self._release()
        return dict(DEFAULT_NAV_PARAMS)

    # -- Compute expected box height from target + camera --
    def compute_expected_box_height(self, distance_m):
        """Given target real height, camera FOV, and distance, predict bbox height in pixels.

        This links LiDAR distance measurements to camera calibration:
          bbox_height = (real_height * focal_length) / distance
          focal_length = (frame_height / 2) / tan(vfov / 2)
        """
        if self._try_lock():
            try:
                real_h = self._target['height']
                fov = self._camera_fov
                fh = self._frame_h
                fw = self._frame_w
            finally: self._release()
        else:
            real_h = DEFAULT_TARGET['height']
            fov = DEFAULT_CAMERA_FOV
            fh = DEFAULT_FRAME_H
            fw = DEFAULT_FRAME_W

        if distance_m <= 0:
            return 0.0
        # Vertical FOV approximation (assume 4:3 aspect)
        vfov_rad = math.radians(fov) * (fh / fw)
        focal_px = (fh / 2.0) / math.tan(vfov_rad / 2.0)
        return (real_h * focal_px) / distance_m

    # -- Persistence --
    def save_calibration(self):
        """Save all calibration data to disk (atomic write via temp file).

        Writes to ``~/.config/bowling_target_nav/calibration.json``.
        Uses write-to-tmp + os.replace for crash safety.

        Returns:
            True on success, False on lock timeout or I/O error.
        """
        if self._try_lock():
            try:
                data = {
                    'ref_box_height': self._ref_box_height,
                    'ref_distance': self._ref_distance,
                    'confidence_threshold': self._confidence_threshold,
                    'detect_interval': self._detect_interval,
                    'detect_expiry': self._detect_expiry,
                    'filter_max_aspect': self._filter_max_aspect,
                    'filter_max_box_pct': self._filter_max_box_pct,
                    'filter_size_tolerance': self._filter_size_tolerance,
                    'filter_tracker_hits': self._filter_tracker_hits,
                    'filter_tracker_dist': self._filter_tracker_dist,
                    'filter_bbox_window': self._filter_bbox_window,
                    'filter_bbox_factor': self._filter_bbox_factor,
                    'robot': dict(self._robot),
                    'lidar': dict(self._lidar),
                    'lidar_yaw': self._lidar_yaw,
                    'camera_pos': dict(self._camera),
                    'camera_yaw': self._camera_yaw,
                    'camera_fov': self._camera_fov,
                    'target': dict(self._target),
                    'ref_point': self._ref_point,
                    'nav_params': dict(self._nav_params),
                    'map_params': dict(self._map_params),
                }
            finally: self._release()
        else:
            return False
        try:
            cal_dir = os.path.dirname(CALIBRATION_FILE)
            os.makedirs(cal_dir, exist_ok=True)
            tmp_file = CALIBRATION_FILE + '.tmp'
            with open(tmp_file, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp_file, CALIBRATION_FILE)
            print(f"[Calibration] Saved to {CALIBRATION_FILE}", flush=True)
            return True
        except Exception as e:
            print(f"[Calibration] Save failed: {e}", flush=True)
            return False

    @staticmethod
    def _clamp_positive(val, minimum=0.001):
        """Clamp a value to be at least minimum (prevents div-by-zero)."""
        try:
            v = float(val)
            return v if v >= minimum else minimum
        except (TypeError, ValueError):
            return minimum

    def _load_calibration(self):
        """Load calibration from disk if it exists, with validation.

        Called once at __init__ and again when settings_changed events fire
        in multiprocess mode.  Values are clamped to safe ranges to
        prevent division-by-zero or unreasonable behavior.
        """
        try:
            if os.path.exists(CALIBRATION_FILE):
                with open(CALIBRATION_FILE, 'r') as f:
                    data = json.load(f)
                self._ref_box_height = self._clamp_positive(
                    data.get('ref_box_height', self._ref_box_height), 1.0)
                self._ref_distance = self._clamp_positive(
                    data.get('ref_distance', self._ref_distance), 0.01)
                if 'confidence_threshold' in data:
                    self._confidence_threshold = max(0.05, min(0.95, float(data['confidence_threshold'])))
                if 'detect_interval' in data:
                    self._detect_interval = max(0.0, float(data['detect_interval']))
                if 'detect_expiry' in data:
                    self._detect_expiry = max(0.3, float(data['detect_expiry']))
                if 'filter_max_aspect' in data:
                    self._filter_max_aspect = max(0.5, min(3.0, float(data['filter_max_aspect'])))
                if 'filter_max_box_pct' in data:
                    self._filter_max_box_pct = max(0.4, min(1.0, float(data['filter_max_box_pct'])))
                if 'filter_size_tolerance' in data:
                    self._filter_size_tolerance = max(1.0, min(5.0, float(data['filter_size_tolerance'])))
                if 'filter_tracker_hits' in data:
                    self._filter_tracker_hits = max(1, min(10, int(data['filter_tracker_hits'])))
                if 'filter_tracker_dist' in data:
                    self._filter_tracker_dist = max(10, min(200, int(data['filter_tracker_dist'])))
                if 'filter_bbox_window' in data:
                    self._filter_bbox_window = max(3, min(50, int(data['filter_bbox_window'])))
                if 'filter_bbox_factor' in data:
                    self._filter_bbox_factor = max(1.2, min(10.0, float(data['filter_bbox_factor'])))
                if 'robot' in data:
                    self._robot.update(data['robot'])
                    # Validate robot dims
                    for k in ('length', 'width', 'height'):
                        self._robot[k] = self._clamp_positive(self._robot.get(k, 0.1), 0.01)
                if 'lidar' in data:
                    self._lidar.update(data['lidar'])
                if 'lidar_yaw' in data:
                    self._lidar_yaw = float(data['lidar_yaw'])
                if 'camera_pos' in data:
                    self._camera = dict(data['camera_pos'])
                if 'camera_yaw' in data:
                    self._camera_yaw = float(data['camera_yaw'])
                if 'camera_fov' in data:
                    self._camera_fov = max(10.0, min(180.0, float(data['camera_fov'])))
                if 'target' in data:
                    self._target.update(data['target'])
                    self._target['height'] = self._clamp_positive(self._target.get('height', 0.25), 0.01)
                    self._target['width'] = self._clamp_positive(self._target.get('width', 0.07), 0.01)
                if data.get('ref_point') in REF_POINTS:
                    self._ref_point = data['ref_point']
                if 'nav_params' in data:
                    for k, v in data['nav_params'].items():
                        if k in DEFAULT_NAV_PARAMS:
                            self._nav_params[k] = type(DEFAULT_NAV_PARAMS[k])(v)
                    # Validate critical nav params
                    for k in ('obstacle_distance', 'linear_speed', 'min_linear_speed',
                              'angular_speed', 'blind_approach_speed'):
                        if k in self._nav_params:
                            self._nav_params[k] = max(0.01, self._nav_params[k])
                    if self._nav_params.get('approach_distance', 0) < 0:
                        self._nav_params['approach_distance'] = 0.0
                if 'map_params' in data:
                    for k, v in data['map_params'].items():
                        if k in DEFAULT_MAP_PARAMS:
                            self._map_params[k] = type(DEFAULT_MAP_PARAMS[k])(v)
                self._calibration_dirty = False  # Just loaded from disk, not dirty
                print(f"[Calibration] Loaded: ref={self._ref_box_height}px@{self._ref_distance}m "
                      f"robot={self._robot} lidar={self._lidar} target={self._target}", flush=True)
        except Exception as e:
            print(f"[Calibration] Load failed: {e}", flush=True)

    # -- Stream/detector mode --
    def set_stream_mode(self, enabled):
        if self._try_lock():
            try: self._stream_mode = enabled
            finally: self._release()

    def get_stream_mode(self):
        if self._try_lock():
            try: return self._stream_mode
            finally: self._release()
        return False

    def set_detector_mode(self, mode):
        if self._try_lock():
            try: self._detector_mode = mode
            finally: self._release()

    def get_detector_mode(self):
        if self._try_lock():
            try: return self._detector_mode
            finally: self._release()
        return "Initializing"
