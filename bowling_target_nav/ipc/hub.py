"""IPCHub — central container for all inter-process shared primitives.

Created once by the main (GUI) process and passed to child processes via
``multiprocessing.Process(args=(hub,))``.  Because fork inherits file
descriptors and mp.Value/Array handles, no pickling is needed.

IPC architecture (3 processes, one per CPU core):
    Core 0 — GUI (GTK):  reads sensors/detections/nav, writes commands/settings.
    Core 1 — ROS+Nav:    reads commands, writes sensors/nav, reads detections.
    Core 2 — Camera:     writes frames/detections, reads settings.

Data channels:
    SharedMemory  — large blobs (frame 921 KB, map up to 12 MB, laser 16 KB).
    mp.Array/Value — small scalars (pose, nav state, detection structs, diag).
    mp.Queue       — variable-size messages (commands, error strings).
    mp.Event       — one-bit flags (shutdown, emergency_stop, settings_changed).

Lock ordering (acquire in this order to avoid deadlocks):
    1. frame_lock
    2. map_lock
    3. laser_lock
    4. pose_lock
    5. nav_lock
    6. det_lock
    7. diag_lock
In practice each lock protects an independent channel, so nested locking
does not occur.  The ordering is documented here as a safety net.
"""

import multiprocessing as mp
from multiprocessing.shared_memory import SharedMemory
from queue import Empty as QueueEmpty
import time

import numpy as np

# Nav state enum mapping
NAV_STATES = {
    'IDLE': 0, 'NAVIGATING': 1, 'SEARCHING': 2, 'ARRIVED': 3,
    'SPIRAL_SEARCH': 4, 'BLIND_APPROACH': 5, 'UNKNOWN': 6, 'ERROR': 7,
}
NAV_STATES_REV = {v: k for k, v in NAV_STATES.items()}

# Detection layout: per detection = 8 doubles
# [confidence, x1, y1, x2, y2, distance, angle, bbox_clipped]
MAX_DETECTIONS = 20
DET_FIELDS = 8
DET_HEADER = 2  # [n_detections, timestamp]

# Max class_name length per detection (UTF-8 bytes)
MAX_CLASS_NAME_LEN = 32


class IPCHub:
    """All shared IPC primitives for the 3-process architecture.

    Process layout:
        - GUI (main): reads sensors/detections/nav, writes commands/settings
        - ROS+Nav: reads commands, writes sensors/nav, reads detections
        - Camera: writes frames/detections, reads settings

    Shared memory regions:
        - frame_shm: camera frame (640*480*3 = 921,600 bytes)
        - map_shm: SLAM map (up to 2048*2048*3 = 12,582,912 bytes)
        - laser_shm: laser points (up to 2000*2*4 = 16,000 bytes)

    mp.Array/Value:
        - pose: robot pose [x, y, theta, timestamp]
        - nav_state: nav state enum + nav_data array
        - det_data: packed detection structs
        - det_info: detection info string
        - det_class_names: class names for each detection
        - detector_mode: detector mode string
        - diag: diagnostics data

    mp.Queue:
        - cmd_queue: GUI → ROS commands
        - error_queue: any process → GUI errors

    mp.Event:
        - shutdown: global shutdown signal
        - emergency_stop: safety-critical stop (never dropped)
        - settings_changed_ros/cam: per-process calibration reload
    """

    # Size constants
    FRAME_W = 640
    FRAME_H = 480
    FRAME_SIZE = FRAME_W * FRAME_H * 3  # 921,600 bytes

    MAX_MAP_W = 2048
    MAX_MAP_H = 2048
    MAX_MAP_SIZE = MAX_MAP_W * MAX_MAP_H * 3  # ~12MB

    MAX_LASER_POINTS = 2000
    MAX_LASER_SIZE = MAX_LASER_POINTS * 2 * 4  # 16KB (float32 x,y pairs)

    def __init__(self):
        """Allocate all shared memory regions, locks, arrays, queues, and events.

        Must be called in the main (GUI) process before forking children.
        """
        # ---- Events ----
        self.shutdown = mp.Event()
        self.emergency_stop = mp.Event()  # Safety-critical: never dropped by queue
        # Separate settings events per consumer (each process clears its own)
        self.settings_changed_ros = mp.Event()
        self.settings_changed_cam = mp.Event()
        self.bbox_reset_cam = mp.Event()  # Signal camera to reset bbox tracker on GO

        # ---- Frame Channel (Camera → GUI) ----
        self.frame_shm = SharedMemory(create=True, size=self.FRAME_SIZE)
        self.frame_lock = mp.Lock()
        self.frame_seq = mp.Value('i', 0)
        self.frame_w = mp.Value('i', 0)
        self.frame_h = mp.Value('i', 0)

        # ---- Map Channel (ROS → GUI) ----
        self.map_shm = SharedMemory(create=True, size=self.MAX_MAP_SIZE)
        self.map_lock = mp.Lock()
        self.map_seq = mp.Value('i', 0)
        self.map_w = mp.Value('i', 0)
        self.map_h = mp.Value('i', 0)
        # [resolution, origin_x, origin_y]
        self.map_info = mp.Array('d', [0.05, 0.0, 0.0])

        # ---- Laser Channel (ROS → GUI) ----
        self.laser_shm = SharedMemory(create=True, size=self.MAX_LASER_SIZE)
        self.laser_lock = mp.Lock()
        self.laser_n = mp.Value('i', 0)
        self.laser_seq = mp.Value('i', 0)
        self.laser_time = mp.Value('d', 0.0)

        # ---- Robot Pose (ROS → GUI) ----
        # [x, y, theta, timestamp]
        self.pose = mp.Array('d', [0.0, 0.0, 0.0, 0.0])
        self.pose_lock = mp.Lock()

        # ---- Nav State (ROS ↔ GUI) ----
        self.nav_lock = mp.Lock()
        self.nav_state = mp.Value('i', 0)  # NAV_STATES enum
        # [0:3] = nav_target (x, y, dist)
        # [3:5] = nav_target_map (x, y)
        # [5:8] = cmd_vel (vx, vy, wz)
        # [8]   = time_since_target
        # [9]   = search_time
        # [10]  = obstacle_dist
        # [11]  = obstacle_ahead (0/1)
        # [12]  = lidar_at_target (distance at target angle)
        self.nav_data = mp.Array('d', [0.0] * 13)

        # ---- Detections (Camera → ROS+GUI) ----
        # Header [n_dets, timestamp] + MAX_DETECTIONS * 8 doubles each
        self.det_data = mp.Array('d', [0.0] * (DET_HEADER + MAX_DETECTIONS * DET_FIELDS))
        self.det_lock = mp.Lock()
        self.det_info = mp.Array('c', b'\x00' * 256)
        # Class names: MAX_DETECTIONS slots of MAX_CLASS_NAME_LEN bytes each
        self.det_class_names = mp.Array('c', b'\x00' * (MAX_DETECTIONS * MAX_CLASS_NAME_LEN))
        self.detector_mode = mp.Array('c', b'\x00' * 64)

        # ---- Queues ----
        self.cmd_queue = mp.Queue(maxsize=20)
        self.error_queue = mp.Queue(maxsize=50)

        # ---- Diagnostics (ROS → GUI) ----
        # [scan_hz, map_hz, scan_count, map_count, scan_age, map_age, tf_age, n_points]
        self.diag = mp.Array('d', [0.0] * 8)
        self.diag_lock = mp.Lock()

    # ==================================================================
    # Frame read/write
    # ==================================================================

    def write_frame(self, frame_rgb):
        """Write camera frame to shared memory (Camera process).

        Args:
            frame_rgb: numpy uint8 array of shape (H, W, 3) in RGB order.
                       Frames larger than FRAME_SIZE are silently dropped.
        """
        if frame_rgb is None:
            return
        h, w = frame_rgb.shape[:2]
        nbytes = h * w * 3
        if nbytes > self.FRAME_SIZE:
            return
        with self.frame_lock:
            dst = np.ndarray((h, w, 3), dtype=np.uint8,
                             buffer=self.frame_shm.buf)
            np.copyto(dst, frame_rgb)  # handles non-contiguous (BGR→RGB view)
            self.frame_w.value = w
            self.frame_h.value = h
            self.frame_seq.value += 1

    def read_frame(self):
        """Read camera frame from shared memory (GUI process).

        Returns (frame_rgb, seq) or (None, seq).
        """
        with self.frame_lock:
            w = self.frame_w.value
            h = self.frame_h.value
            seq = self.frame_seq.value
            if w == 0 or h == 0:
                return None, seq
            src = np.ndarray((h, w, 3), dtype=np.uint8,
                             buffer=self.frame_shm.buf)
            frame = src.copy()  # single memcpy
        return frame, seq

    # ==================================================================
    # Map read/write
    # ==================================================================

    def write_map(self, map_img, map_info):
        """Write SLAM map to shared memory (ROS process).

        Args:
            map_img: BGR numpy array (h, w, 3) uint8
            map_info: ROS MapMetaData or MapInfo with resolution, origin
        """
        if map_img is None:
            return
        h, w = map_img.shape[:2]
        if w > self.MAX_MAP_W or h > self.MAX_MAP_H:
            return
        nbytes = h * w * 3
        # Extract origin before lock
        if hasattr(map_info, 'origin'):
            if hasattr(map_info.origin, 'position'):
                ox = map_info.origin.position.x
                oy = map_info.origin.position.y
            else:
                ox = getattr(map_info, 'origin_x', 0.0)
                oy = getattr(map_info, 'origin_y', 0.0)
        else:
            ox = getattr(map_info, 'origin_x', 0.0)
            oy = getattr(map_info, 'origin_y', 0.0)
        res = getattr(map_info, 'resolution', 0.05)
        with self.map_lock:
            dst = np.ndarray((h, w, 3), dtype=np.uint8,
                             buffer=self.map_shm.buf)
            np.copyto(dst, map_img)
            self.map_w.value = w
            self.map_h.value = h
            self.map_seq.value += 1
            self.map_info[0] = res
            self.map_info[1] = ox
            self.map_info[2] = oy

    def read_map_seq(self):
        """Return current map sequence number without copying data."""
        return self.map_seq.value

    def read_map(self):
        """Read SLAM map from shared memory (GUI process).

        Returns (map_img, MapInfo, seq) or (None, None, seq).
        """
        from .map_info import MapInfo
        with self.map_lock:
            w = self.map_w.value
            h = self.map_h.value
            seq = self.map_seq.value
            if w == 0 or h == 0:
                return None, None, seq
            res = self.map_info[0]
            ox = self.map_info[1]
            oy = self.map_info[2]
            src = np.ndarray((h, w, 3), dtype=np.uint8,
                             buffer=self.map_shm.buf)
            img = src.copy()
        info = MapInfo(resolution=res, origin_x=ox, origin_y=oy,
                       width=w, height=h)
        return img, info, seq

    # ==================================================================
    # Laser read/write
    # ==================================================================

    def write_laser(self, points, laser_time=None):
        """Write laser points to shared memory (ROS process).

        Args:
            points: numpy float32 array shape (N, 2)
            laser_time: timestamp (defaults to time.time())
        """
        if points is None or len(points) == 0:
            with self.laser_lock:
                self.laser_n.value = 0
                self.laser_seq.value += 1
                self.laser_time.value = laser_time or time.time()
            return
        n = min(len(points), self.MAX_LASER_POINTS)
        pts = np.ascontiguousarray(points[:n], dtype=np.float32)
        data = pts.tobytes()  # copy outside lock
        nbytes = n * 2 * 4
        with self.laser_lock:
            self.laser_shm.buf[:nbytes] = data
            self.laser_n.value = n
            self.laser_seq.value += 1
            self.laser_time.value = laser_time or time.time()

    def read_laser_seq(self):
        """Return current laser sequence number without copying data."""
        return self.laser_seq.value

    def read_laser(self):
        """Read laser points from shared memory (GUI process).

        Returns (points, seq, laser_time).
        """
        with self.laser_lock:
            n = self.laser_n.value
            seq = self.laser_seq.value
            lt = self.laser_time.value
            if n == 0:
                return np.empty((0, 2), dtype=np.float32), seq, lt
            nbytes = n * 2 * 4
            buf = bytes(self.laser_shm.buf[:nbytes])
        return np.frombuffer(buf, dtype=np.float32).reshape(n, 2).copy(), seq, lt

    # ==================================================================
    # Pose read/write
    # ==================================================================

    def write_pose(self, x, y, theta):
        """Write robot pose (ROS process).

        Args:
            x: Robot X position in map frame (meters).
            y: Robot Y position in map frame (meters).
            theta: Robot heading (radians).
        """
        with self.pose_lock:
            self.pose[0] = x
            self.pose[1] = y
            self.pose[2] = theta
            self.pose[3] = time.time()

    def read_pose(self):
        """Read robot pose (GUI process). Returns (x, y, theta)."""
        with self.pose_lock:
            return self.pose[0], self.pose[1], self.pose[2]

    # ==================================================================
    # Nav state read/write
    # ==================================================================

    def write_nav_state(self, state_str, nav_target=None, nav_target_map=None,
                        cmd_vel=None, time_since_target=999.0,
                        search_time=0.0, obstacle_dist=float('inf'),
                        obstacle_ahead=False, lidar_at_target=float('inf')):
        """Write full navigation state (ROS process).

        Args:
            state_str: One of NAV_STATES keys (e.g. 'IDLE', 'NAVIGATING').
            nav_target: (x, y[, distance]) in robot frame, or None.
            nav_target_map: (x, y) in map frame for GUI overlay, or None.
            cmd_vel: (vx, vy, wz) current velocity command, or None.
            time_since_target: Seconds since last target detection.
            search_time: Seconds spent in current search phase.
            obstacle_dist: Distance to nearest obstacle (meters).
            obstacle_ahead: True if an obstacle blocks the forward path.
            lidar_at_target: LiDAR distance at target angle (meters).
        """
        with self.nav_lock:
            self.nav_state.value = NAV_STATES.get(state_str, 0)
            if nav_target is not None:
                self.nav_data[0] = nav_target[0]
                self.nav_data[1] = nav_target[1]
                self.nav_data[2] = nav_target[2] if len(nav_target) > 2 else 0.0
            if nav_target_map is not None:
                self.nav_data[3] = nav_target_map[0]
                self.nav_data[4] = nav_target_map[1]
            else:
                self.nav_data[3] = float('nan')
                self.nav_data[4] = float('nan')
            if cmd_vel is not None:
                self.nav_data[5] = cmd_vel[0]
                self.nav_data[6] = cmd_vel[1]
                self.nav_data[7] = cmd_vel[2]
            self.nav_data[8] = time_since_target
            self.nav_data[9] = search_time
            self.nav_data[10] = obstacle_dist
            self.nav_data[11] = 1.0 if obstacle_ahead else 0.0
            self.nav_data[12] = lidar_at_target

    def read_nav_snapshot(self):
        """Read full navigation state (GUI process).

        Returns dict matching NavStore.get_gui_snapshot() format.
        """
        import math
        with self.nav_lock:
            state_str = NAV_STATES_REV.get(self.nav_state.value, 'UNKNOWN')
            d0, d1, d2 = self.nav_data[0], self.nav_data[1], self.nav_data[2]
            d3, d4 = self.nav_data[3], self.nav_data[4]
            d5, d6, d7 = self.nav_data[5], self.nav_data[6], self.nav_data[7]
            d8, d9, d10, d11 = self.nav_data[8], self.nav_data[9], self.nav_data[10], self.nav_data[11]
            d12 = self.nav_data[12]
        nav_target = (d0, d1, d2) if state_str != 'IDLE' else None
        nav_map = (d3, d4) if math.isfinite(d3) and math.isfinite(d4) else None
        return {
            'nav_state': state_str,
            'nav_target': nav_target,
            'time_since_target': d8,
            'search_time': d9,
            'obstacle_ahead': d11 > 0.5,
            'obstacle_dist': d10,
            'cmd_vel': (d5, d6, d7),
            'nav_target_map': nav_map,
            'lidar_at_target': d12,
        }

    # ==================================================================
    # Detection read/write
    # ==================================================================

    def write_detections(self, detections, info_str, timestamp=None):
        """Write detection list to shared memory (Camera process).

        Each detection dict: confidence, bbox(4), distance, angle, bbox_clipped, class_name.
        """
        ts = timestamp or time.time()
        n = min(len(detections), MAX_DETECTIONS)
        with self.det_lock:
            self.det_data[0] = float(n)
            self.det_data[1] = ts
            for i in range(n):
                det = detections[i]
                base = DET_HEADER + i * DET_FIELDS
                self.det_data[base] = det.get('confidence', 0.0)
                bbox = det.get('bbox', (0, 0, 0, 0))
                self.det_data[base + 1] = float(bbox[0])
                self.det_data[base + 2] = float(bbox[1])
                self.det_data[base + 3] = float(bbox[2])
                self.det_data[base + 4] = float(bbox[3])
                self.det_data[base + 5] = det.get('distance', 0.0)
                self.det_data[base + 6] = det.get('angle', 0.0)
                self.det_data[base + 7] = 1.0 if det.get('bbox_clipped', False) else 0.0
                # Class name
                cls = det.get('class_name', '').encode('utf-8')[:MAX_CLASS_NAME_LEN - 1]
                offset = i * MAX_CLASS_NAME_LEN
                padded = cls.ljust(MAX_CLASS_NAME_LEN, b'\x00')
                self.det_class_names.raw = (
                    self.det_class_names.raw[:offset] + padded +
                    self.det_class_names.raw[offset + MAX_CLASS_NAME_LEN:])
            # Info string
            encoded = info_str.encode('utf-8')[:255]
            self.det_info.raw = encoded.ljust(256, b'\x00')

    def read_detections(self):
        """Read detection list (ROS+GUI process).

        Returns (detections_list, info_str, timestamp).
        """
        with self.det_lock:
            n = min(int(self.det_data[0]), MAX_DETECTIONS)
            ts = self.det_data[1]
            dets = []
            raw_names = self.det_class_names.raw
            for i in range(n):
                base = DET_HEADER + i * DET_FIELDS
                # Extract class_name
                offset = i * MAX_CLASS_NAME_LEN
                cls_bytes = raw_names[offset:offset + MAX_CLASS_NAME_LEN]
                cls_name = cls_bytes.rstrip(b'\x00').decode('utf-8', errors='replace')
                dets.append({
                    'confidence': self.det_data[base],
                    'bbox': (int(self.det_data[base + 1]),
                             int(self.det_data[base + 2]),
                             int(self.det_data[base + 3]),
                             int(self.det_data[base + 4])),
                    'distance': self.det_data[base + 5],
                    'angle': self.det_data[base + 6],
                    'bbox_clipped': self.det_data[base + 7] > 0.5,
                    'class_name': cls_name,
                })
            info = self.det_info.raw.rstrip(b'\x00').decode('utf-8', errors='replace')
        return dets, info, ts

    def write_detector_mode(self, mode_str):
        """Write detector mode string (Camera process)."""
        with self.det_lock:
            encoded = mode_str.encode('utf-8')[:63]
            self.detector_mode.raw = encoded.ljust(64, b'\x00')

    def read_detector_mode(self):
        """Read detector mode string (GUI process)."""
        with self.det_lock:
            return self.detector_mode.raw.rstrip(b'\x00').decode('utf-8', errors='replace')

    # ==================================================================
    # Diagnostics
    # ==================================================================

    def write_diagnostics(self, scan_hz=0.0, map_hz=0.0, scan_count=0,
                          map_count=0, scan_age=-1.0, map_age=-1.0,
                          tf_age=-1.0, n_points=0):
        """Write diagnostic data (ROS process).

        Args:
            scan_hz: Smoothed LiDAR scan receive rate.
            map_hz: Smoothed SLAM map receive rate.
            scan_count: Total scans received.
            map_count: Total map updates received.
            scan_age: Seconds since last scan (-1 if never).
            map_age: Seconds since last map (-1 if never).
            tf_age: Seconds since last TF pose (-1 if never).
            n_points: Number of laser points in latest scan.
        """
        with self.diag_lock:
            self.diag[0] = scan_hz
            self.diag[1] = map_hz
            self.diag[2] = float(scan_count)
            self.diag[3] = float(map_count)
            self.diag[4] = scan_age
            self.diag[5] = map_age
            self.diag[6] = tf_age
            self.diag[7] = float(n_points)

    def read_diagnostics(self):
        """Read diagnostic data (GUI process).

        Returns:
            dict with keys: scan_hz, map_hz, scan_count, map_count,
            scan_age, map_age, tf_age, n_points.
        """
        with self.diag_lock:
            return {
                'scan_hz': self.diag[0],
                'map_hz': self.diag[1],
                'scan_count': int(self.diag[2]),
                'map_count': int(self.diag[3]),
                'scan_age': self.diag[4],
                'map_age': self.diag[5],
                'tf_age': self.diag[6],
                'n_points': int(self.diag[7]),
            }

    # ==================================================================
    # Error queue
    # ==================================================================

    def add_error(self, source, error):
        """Add error message from any process."""
        try:
            self.error_queue.put_nowait(f"{source}: {error}")
        except Exception:
            pass

    def get_errors(self):
        """Drain all error messages (GUI process)."""
        errors = []
        while True:
            try:
                errors.append(self.error_queue.get_nowait())
            except (QueueEmpty, OSError):
                break
        return errors

    # ==================================================================
    # Cleanup
    # ==================================================================

    def close_child(self):
        """Close SharedMemory handles in child process (without unlink)."""
        for shm in [self.frame_shm, self.map_shm, self.laser_shm]:
            try:
                shm.close()
            except Exception:
                pass

    def cleanup(self):
        """Release SharedMemory (call from main process only)."""
        for shm in [self.frame_shm, self.map_shm, self.laser_shm]:
            try:
                shm.close()
                shm.unlink()
            except Exception:
                pass
        # Close queues
        for q in [self.cmd_queue, self.error_queue]:
            try:
                q.close()
                q.join_thread()
            except Exception:
                pass
