"""Kalman-filtered target tracker with size-distance gating and LiDAR fusion.

Provides TargetTracker, which sits between the raw DRP-AI detection pipeline
and the navigation controller. It validates, matches, and smooths detections
to produce a stable, continuous target position.

Pipeline per tick (20 Hz):
    1. Size-Distance Gate: reject detections whose bbox height doesn't match
       the expected height for the measured distance (physics-based filter).
    2. LiDAR Match: find the LiDAR obstacle closest to the camera detection
       angle and distance. Use LiDAR distance when matched (more accurate).
    3. Kalman Filter: smooth and predict position during detection drops.

Kalman Filter Details:
    State vector (4D): [distance, angle, d_velocity, a_velocity]
        - distance:   straight-line distance to target (meters)
        - angle:      bearing in base_link frame (radians, positive = left)
        - d_velocity: rate of change of distance (m/s, negative = approaching)
        - a_velocity: rate of change of angle (rad/s)

    Process model: constant-velocity (naive integration)
        x_predicted = x + velocity * dt

    Measurement: [distance, angle] from camera (or LiDAR-fused distance)
        - Camera gives both distance (from pinhole model) and angle
        - LiDAR gives more accurate distance when matched (lower noise R)

    Noise tuning (DEFAULT_TRACKER_PARAMS in config.py):
        - kf_q_* (process noise): higher = trusts measurements more
        - kf_r_cam_* (camera noise): higher = trusts model/prediction more
        - kf_r_lidar_* (LiDAR noise): lower than camera = prefers LiDAR

    Prediction: when no detection, Kalman predicts position using velocity
        model for up to predict_timeout (3.0s). After that, track is dropped.

Size-Distance Gate:
    Expected bbox height = ref_box_height * (ref_distance / measured_distance)
    If actual bbox height differs by more than size_gate_tolerance (50%),
    the detection is rejected. This filters false positives like chair legs
    or wall edges that have wrong size for their distance.

LiDAR Matching:
    For each camera detection, search LiDAR points within ±lidar_angle_window
    (10°) of the detection bearing. If a LiDAR point is within
    lidar_dist_tolerance (50%) of the camera distance, use the LiDAR
    distance instead (more accurate, lower measurement noise).

Why in Python, not C++:
    The Kalman filter needs LiDAR data (for matching) and runs between camera
    frames (for prediction). The C++ DRP-AI binary only sees camera pixels.
    The filter runs at 20 Hz (control loop rate) and takes ~100us per tick
    on ARM — performance is not a concern.

Thread safety:
    Called only from NavController.step() in the ROS timer thread.
    No internal locks needed.

Key classes:
    TrackedTarget -- output dataclass with smoothed position.
    TargetTracker -- the tracker with gate, match, and Kalman logic.

Related modules:
    app/nav_controller.py  -- calls tracker.update() each tick.
    nav/target_selector.py -- find_best_target() picks the highest-priority
                              detection from the raw list before tracking.
    config.py              -- DEFAULT_TRACKER_PARAMS, calibration defaults.
"""

import math
import time
from dataclasses import dataclass

import numpy as np

from target_nav.config import (
    DEFAULT_CAMERA, DEFAULT_FRAME_W, DEFAULT_REF_BOX_HEIGHT,
    DEFAULT_REF_DISTANCE, DEFAULT_TARGET, DEFAULT_TRACKER_PARAMS,
    LIDAR_MIN_RANGE,
)
from target_nav.nav.target_selector import find_best_target
from target_nav.utils.log import get_logger

logger = get_logger('app.target_tracker')


@dataclass(slots=True)
class TrackedTarget:
    """Output of TargetTracker -- a smoothed, validated target position.

    All coordinates are in the robot's base_link frame:
        x_bl: forward distance (positive = ahead of robot center)
        y_bl: lateral distance (positive = left of robot center)
        distance: straight-line distance to target
        angle: bearing in base_link frame (positive = left, ROS convention)
    """
    distance: float
    angle: float
    x_bl: float
    y_bl: float
    confidence: float
    lidar_confirmed: bool
    age_ticks: int
    bbox_clipped: bool


class TargetTracker:
    """Size-gated, LiDAR-confirmed, Kalman-filtered single-target tracker.

    Usage::

        tracker = TargetTracker()

        # In 20 Hz control loop:
        tracked = tracker.update(detections, lpoints, camera_yaw_rad)
        if tracked:
            # Use tracked.distance, tracked.angle, tracked.x_bl, tracked.y_bl
            ...

        # On GO/STOP:
        tracker.reset()
    """

    def __init__(self):
        """Initialize Kalman state, preallocate numpy arrays, load config."""
        p = DEFAULT_TRACKER_PARAMS

        # --- Config (refreshed periodically) ---
        self._ref_box_height = DEFAULT_REF_BOX_HEIGHT
        self._ref_distance = DEFAULT_REF_DISTANCE
        self._size_gate_tol = p['size_gate_tolerance']
        self._lidar_angle_window = math.radians(p['lidar_angle_window'])
        self._lidar_dist_tol = p['lidar_dist_tolerance']
        # Camera position for LiDAR-camera distance matching
        self._cam_x = DEFAULT_CAMERA['x']
        self._cam_y = DEFAULT_CAMERA['y']
        self._predict_timeout = p['predict_timeout']
        self._predict_max_ticks = int(p['predict_timeout'] * 20)  # at 20 Hz
        self._debug_counter = 0

        # --- Kalman matrices (preallocated, reused every tick) ---
        # State: [distance, angle, d_velocity, a_velocity]
        self._x = np.zeros(4, dtype=np.float64)      # state vector
        self._P = np.eye(4, dtype=np.float64)         # covariance
        self._F = np.eye(4, dtype=np.float64)         # state transition
        self._H = np.zeros((2, 4), dtype=np.float64)  # measurement
        self._H[0, 0] = 1.0  # observe distance
        self._H[1, 1] = 1.0  # observe angle
        self._I = np.eye(4, dtype=np.float64)         # identity

        # Process noise base (scaled by dt each tick)
        self._q = np.array([
            p['kf_q_dist'], p['kf_q_angle'],
            p['kf_q_ddot'], p['kf_q_adot']
        ], dtype=np.float64)

        # Measurement noise: camera-only vs LiDAR-confirmed
        self._R_cam = np.diag([p['kf_r_cam_dist'], p['kf_r_cam_angle']])
        self._R_lidar = np.diag([p['kf_r_lidar_dist'], p['kf_r_cam_angle']])

        # --- Tracking state ---
        self._active = False        # True after first valid detection
        self._age_ticks = 0         # ticks since last real measurement
        self._last_time = 0.0       # timestamp of last update
        self._last_bbox_clipped = False
        self._last_lidar_confirmed = False
        self._last_lidar_dist = float('inf')

        # --- Config refresh timer ---
        self._last_config_refresh = 0.0

    def reset(self):
        """Clear track state. Call on GO/STOP or session start."""
        self._active = False
        self._age_ticks = 0
        self._last_time = 0.0
        self._x[:] = 0.0
        self._P[:] = np.eye(4)
        self._last_bbox_clipped = False
        self._last_lidar_confirmed = False
        self._last_lidar_dist = float('inf')

    def refresh_config(self, settings):
        """Reload calibration values from settings store (call every ~2s).

        Args:
            settings: SettingsStore instance with get_calibration(), etc.
        """
        now = time.time()
        if now - self._last_config_refresh < 2.0:
            return
        self._last_config_refresh = now
        ref_h, ref_d = settings.get_calibration()
        self._ref_box_height = ref_h
        self._ref_distance = ref_d
        cam_pos = settings.get_camera_pos()
        self._cam_x = cam_pos['x']
        self._cam_y = cam_pos['y']

    def update(self, detections, lpoints, camera_yaw_rad=0.0):
        """Run one tick of the tracking pipeline.

        Args:
            detections: list of Det objects from camera worker.
            lpoints: numpy (N, 2) float32 array of LiDAR points in base_link.
            camera_yaw_rad: camera mounting yaw in radians.

        Returns:
            TrackedTarget if tracking is active, None if no track.
        """
        now = time.time()
        dt = now - self._last_time if self._last_time > 0 else 0.05
        dt = min(dt, 0.5)  # clamp to prevent huge jumps after pause
        self._last_time = now

        # --- Stage 1: Find best detection and validate ---
        best = find_best_target(detections)
        measurement = None

        if self._debug_counter < 10 and best is not None:
            logger.info(
                "TRACKER_IN: det=%d best=%.2fm %.1fdeg w=%d h=%d clip=%s pts=%d",
                len(detections), best.distance, math.degrees(best.angle),
                best.width, best.height, best.bbox_clipped, len(lpoints))

        if best is not None:
            # Shape validation using KNOWN target dimensions.
            # Bowling pin: height=0.28m, width=0.08m -> ratio=0.286.
            # At any distance, the pinhole model predicts BOTH expected
            # bbox height AND width. Reject if actual doesn't match.
            #
            # A chair leg might be tall, but its width/height ratio is
            # ~0.08 (very thin) not 0.28 (target-shaped).
            # A chair seat is ~1.5 (wide) — also rejected.
            TARGET_RATIO = DEFAULT_TARGET['width'] / DEFAULT_TARGET['height']
            RATIO_TOL = 0.6  # allow 60% deviation from ideal ratio

            w, h = best.width, best.height
            if w > 0 and h > 0 and not best.bbox_clipped:
                actual_ratio = w / h

                # Check: actual ratio should be close to target ratio (0.28)
                # Acceptable range: 0.28 * (1-0.6) to 0.28 * (1+0.6) = [0.11, 0.45]
                # Chair leg: ~0.08 → too thin, rejected
                # Chair seat: ~1.5 → too wide, rejected
                # Target: ~0.25-0.35 → passes
                ratio_lo = TARGET_RATIO * (1.0 - RATIO_TOL)
                ratio_hi = TARGET_RATIO * (1.0 + RATIO_TOL)
                if actual_ratio < ratio_lo or actual_ratio > ratio_hi:
                    best = None
                # Also check: if distance is known, expected bbox width
                # should match actual width within tolerance
                elif best.distance > 0.1:
                    exp_h = self._ref_box_height * (self._ref_distance / best.distance)
                    exp_w = exp_h * TARGET_RATIO
                    if exp_w > 0 and (w < exp_w * 0.3 or w > exp_w * 3.0):
                        best = None  # width doesn't match target at this distance

            elif best.bbox_clipped and w > 0:
                # Clipped: can't trust height, but width is valid.
                # Check width against expected target width at this distance.
                if best.distance > 0.1:
                    exp_h = self._ref_box_height * (self._ref_distance / best.distance)
                    exp_w = exp_h * TARGET_RATIO
                    if exp_w > 0 and (w < exp_w * 0.3 or w > exp_w * 3.0):
                        best = None

        if best is not None:
            # Convert camera angle to base_link frame
            cam_angle_bl = -best.angle + camera_yaw_rad

            # Size-distance gate (pinhole model validation)
            if self._size_gate(best):
                # --- Stage 2: LiDAR matching ---
                # Use Kalman-filtered distance for matching when available.
                # This is more stable than raw camera distance which flickers.
                # On first detection (no Kalman state), use raw camera distance.
                match_dist = best.distance
                if self._active and self._x[0] > 0.01:
                    match_dist = float(self._x[0])  # Kalman predicted distance

                lidar_dist = self._match_lidar(
                    match_dist, cam_angle_bl, lpoints)

                if self._debug_counter < 10:
                    self._debug_counter += 1
                    logger.info(
                        "TRACKER: dist=%.2f angle=%.1fdeg pts=%d lidar=%.2f",
                        match_dist, math.degrees(cam_angle_bl),
                        len(lpoints), lidar_dist)

                if lidar_dist < float('inf'):
                    # LiDAR confirmed: use LiDAR distance (more accurate)
                    measurement = np.array([lidar_dist, cam_angle_bl])
                    self._last_lidar_confirmed = True
                    self._last_lidar_dist = lidar_dist
                else:
                    # Camera only: use camera distance
                    measurement = np.array([best.distance, cam_angle_bl])
                    self._last_lidar_confirmed = False
                    self._last_lidar_dist = float('inf')

                self._last_bbox_clipped = best.bbox_clipped

        # --- Stage 3: Kalman filter ---
        if measurement is not None:
            if not self._active:
                # First detection: initialize state
                self._x[0] = measurement[0]  # distance
                self._x[1] = measurement[1]  # angle
                self._x[2] = 0.0             # d_velocity
                self._x[3] = 0.0             # a_velocity
                self._P[:] = np.diag([0.5, 0.3, 1.0, 1.0])
                self._active = True
                self._age_ticks = 0
                logger.info("Track initialized: d=%.2fm, a=%.1f°",
                            measurement[0], math.degrees(measurement[1]))
            else:
                # Predict + update
                self._kalman_predict(dt)
                R = self._R_lidar if self._last_lidar_confirmed else self._R_cam
                self._kalman_update(measurement, R)
                self._age_ticks = 0
        elif self._active:
            # No detection: predict only (coast)
            self._kalman_predict(dt)
            self._age_ticks += 1

            # Track lost after timeout
            if self._age_ticks > self._predict_max_ticks:
                self._active = False
                return None

        if not self._active:
            return None

        # --- Build output ---
        dist = max(0.01, float(self._x[0]))
        angle = float(self._x[1])
        confidence = max(0.0, 1.0 - self._age_ticks / max(1, self._predict_max_ticks))

        # Target position in base_link frame.  The Kalman distance is from
        # the camera, so add the camera mounting offset (cam_x, cam_y) to
        # get true base_link coordinates.
        return TrackedTarget(
            distance=dist,
            angle=angle,
            x_bl=self._cam_x + dist * math.cos(angle),
            y_bl=self._cam_y + dist * math.sin(angle),
            confidence=confidence,
            lidar_confirmed=self._last_lidar_confirmed,
            age_ticks=self._age_ticks,
            bbox_clipped=self._last_bbox_clipped,
        )

    def get_lidar_dist(self):
        """Return the last matched LiDAR distance, or inf if no match."""
        return self._last_lidar_dist

    # ------------------------------------------------------------------
    # Internal methods
    # ------------------------------------------------------------------

    def _size_gate(self, det):
        """Validate that bbox height is plausible for the measured distance.

        Uses the calibrated pinhole model: at distance d, the expected bbox
        height is ref_box_height * (ref_distance / d). If the actual bbox
        height deviates more than size_gate_tolerance, the detection is
        rejected as a false positive.

        Clipped bboxes (target extends beyond frame edge) always pass —
        their measured height is artificially smaller than expected because
        the frame truncates the bottom of the bounding box.

        Args:
            det: Det object with distance and bbox (x1, y1, x2, y2).

        Returns:
            True if the detection passes the gate, False otherwise.
        """
        # Clipped bboxes: height is truncated, skip height-based gate.
        # Width-based validation was already done in the shape filter above.
        if det.bbox_clipped:
            return True

        if det.distance <= 0 or not math.isfinite(det.distance):
            return False

        expected_h = self._ref_box_height * (self._ref_distance / det.distance)
        if expected_h <= 0:
            return True

        actual_h = det.height
        if actual_h <= 0:
            return False

        # Allow wider tolerance — the DRP-AI model at low confidence
        # produces imprecise bbox sizes, especially at distance.
        # Ratio 0.3-3.0 catches targets while rejecting most chairs.
        ratio = actual_h / expected_h
        return 0.3 < ratio < 3.0

    def _match_lidar(self, cam_dist, cam_angle_bl, lpoints):
        """Find the LiDAR obstacle matching the camera detection.

        Searches for LiDAR points within an angular and distance window
        around the camera detection. Returns the closest matching distance
        measured from the camera position, or inf if no match found.

        LiDAR points are in base_link frame.  cam_dist is measured from the
        camera sensor.  To compare them correctly, point distances are
        computed relative to the camera position (not base_link origin).

        Args:
            cam_dist: camera-estimated distance in meters (from camera sensor).
            cam_angle_bl: detection angle in base_link frame (radians).
            lpoints: numpy (N, 2) float32 array of (x, y) in base_link.

        Returns:
            LiDAR distance from camera in meters, or float('inf') if no match.
        """
        if len(lpoints) == 0 or cam_dist <= 0:
            return float('inf')

        pts = np.asarray(lpoints, dtype=np.float32)

        # Compute angles from base_link origin for angular matching
        pt_angles = np.arctan2(pts[:, 1], pts[:, 0])

        # Angular filter (wrapping-safe)
        angle_diff = np.abs(np.arctan2(
            np.sin(pt_angles - cam_angle_bl),
            np.cos(pt_angles - cam_angle_bl)))
        angle_mask = angle_diff < self._lidar_angle_window

        if not np.any(angle_mask):
            return float('inf')

        matched_pts = pts[angle_mask]

        # Compute distances from the CAMERA position (not origin)
        # so they're comparable to cam_dist which is from the camera sensor.
        dx = matched_pts[:, 0] - self._cam_x
        dy = matched_pts[:, 1] - self._cam_y
        matched_dists = np.hypot(dx, dy)

        # Distance filter
        dist_lo = cam_dist * (1.0 - self._lidar_dist_tol)
        dist_hi = cam_dist * (1.0 + self._lidar_dist_tol)
        dist_mask = (matched_dists >= max(dist_lo, LIDAR_MIN_RANGE)) & \
                    (matched_dists <= dist_hi)

        if np.any(dist_mask):
            return float(np.min(matched_dists[dist_mask]))

        return float('inf')

    def _kalman_predict(self, dt):
        """Kalman predict step: x = F @ x, P = F @ P @ F.T + Q.

        Uses constant-velocity model in polar coordinates.

        Args:
            dt: time step in seconds.
        """
        # Update state transition matrix with dt
        F = self._F
        F[0, 2] = dt  # distance += d_velocity * dt
        F[1, 3] = dt  # angle += a_velocity * dt

        # Predict state
        self._x = F @ self._x

        # Keep distance positive
        if self._x[0] < 0.01:
            self._x[0] = 0.01

        # Predict covariance
        Q = np.diag(self._q * dt)
        self._P = F @ self._P @ F.T + Q

    def _kalman_update(self, z, R):
        """Kalman update step: fuse measurement with prediction.

        Args:
            z: measurement vector [distance, angle].
            R: measurement noise covariance (2x2).
        """
        H = self._H
        # Innovation
        y = z - H @ self._x
        # Innovation covariance
        S = H @ self._P @ H.T + R
        # Kalman gain
        K = self._P @ H.T @ np.linalg.inv(S)
        # Update state
        self._x = self._x + K @ y
        # Update covariance
        self._P = (self._I - K @ H) @ self._P

        # Keep distance positive
        if self._x[0] < 0.01:
            self._x[0] = 0.01
