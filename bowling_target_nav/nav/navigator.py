"""Mecanum holonomic navigation with obstacle avoidance and blind approach.

This module implements the main navigation pipeline for a 4-wheel mecanum
robot.  It combines:

- **Visual servoing** -- steer toward a camera-detected target.
- **LiDAR+Vision distance fusion** -- use calibrated vision distance by
  default, fall back to LiDAR when the bounding box is clipped.
- **VFH obstacle avoidance** -- Vector Field Histogram gap-finding lets the
  robot strafe around obstacles while keeping the camera on the target.
- **Blind approach** -- dead-reckon to the last known map-frame target
  position when the target drops out of view at close range.
- **Tiered search** -- 360-degree rotation scan (Tier 1) followed by an
  expanding Archimedean spiral (Tier 2) when the target is lost.
- **Temporal arrival confirmation** -- requires multiple sensor signals
  to agree for a sustained period before declaring arrival.

All velocity commands are published through ``_publish_cmd`` which sanitizes
NaN/Inf and enforces speed caps.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist, PointStamped


def _clamp_twist(cmd):
    """Sanitize Twist: replace NaN/Inf with 0, return True if valid."""
    valid = True
    for attr in ('x', 'y', 'z'):
        v = getattr(cmd.linear, attr)
        if not math.isfinite(v):
            setattr(cmd.linear, attr, 0.0)
            valid = False
        v = getattr(cmd.angular, attr)
        if not math.isfinite(v):
            setattr(cmd.angular, attr, 0.0)
            valid = False
    return valid


def _cap_speed(cmd, max_linear, max_angular):
    """Clamp total linear velocity magnitude and angular velocity."""
    total = math.sqrt(cmd.linear.x ** 2 + cmd.linear.y ** 2)
    if total > max_linear and total > 0:
        scale = max_linear / total
        cmd.linear.x *= scale
        cmd.linear.y *= scale
    cmd.angular.z = max(-max_angular, min(max_angular, cmd.angular.z))


class Navigator:
    """Navigation algorithms for mecanum robot.

    Thread-safety: This class is only called from the ROS control_loop timer
    (single-threaded). All shared state access goes through locked stores.

    Args:
        state: SharedState instance for sensor/nav data access.
        cmd_vel_pub: ROS2 Twist publisher for /cmd_vel.
        tf_buffer: TF2 buffer for coordinate transforms.
        nav_client: Optional Nav2 action client.
        logger: ROS2 logger instance.
    """

    def __init__(self, state, cmd_vel_pub, tf_buffer, nav_client, logger):
        self._state = state
        self._cmd_vel_pub = cmd_vel_pub
        self._tf_buffer = tf_buffer
        self._nav_client = nav_client
        self._logger = logger

        # Navigation parameters (tunable via SettingsWindow, persisted in detection_store)
        np = state.detection.get_nav_params()
        self.approach_distance = np['approach_distance']
        self.linear_speed = np['linear_speed']
        self.min_linear_speed = np['min_linear_speed']
        self.angular_speed = np['angular_speed']
        self.obstacle_distance = np['obstacle_distance']
        self.obstacle_slowdown_distance = np['obstacle_slowdown_distance']
        self.robot_half_width = state.detection.get_robot_half_width()
        self.search_angular_speed = np['search_angular_speed']
        self.lost_timeout = np['lost_timeout']
        self.search_timeout = np['search_timeout']

        # Blind approach parameters
        self.blind_approach_active = False
        self.blind_approach_start_time = 0.0
        self.blind_approach_start_pose = (0.0, 0.0, 0.0)
        self.blind_approach_target_map = (0.0, 0.0)
        self.blind_approach_target_distance = 0.0
        self.blind_approach_entry_distance = np['blind_approach_entry_distance']
        self.blind_approach_speed = np['blind_approach_speed']
        self.blind_approach_timeout = np['blind_approach_timeout']
        self.blind_approach_lidar_stop = np['blind_approach_lidar_stop']
        self.blind_approach_arrival_margin = np['blind_approach_arrival_margin']

        # Tracking
        self.last_target_angle = 0.0
        self._last_nav_log = 0.0
        self._last_obs_log = 0.0
        self._stuck_start = 0.0  # For stuck detection

        # VFH obstacle avoidance state
        self._avoid_direction = 0     # -1=right, +1=left, 0=none
        self._avoid_start_time = 0.0
        self._avoid_min_commit = 1.5  # seconds to commit before allowing switch
        self._avoid_clear_count = 0   # consecutive obstacle-free ticks

        # 360° search scan state (Tier 1)
        self._search_start_yaw = 0.0
        self._search_last_yaw = 0.0
        self._search_total_rotated = 0.0
        self._search_initialized = False

        # Spiral search state (Tier 2: after 360° scan fails)
        self._spiral_active = False
        self._spiral_start_time = 0.0
        self._spiral_start_x = 0.0
        self._spiral_start_y = 0.0
        self._spiral_angle = 0.0
        self._spiral_radius = 0.0
        self._spiral_last_time = 0.0

        # Spiral search tunable parameters
        self.spiral_search_enabled = np['spiral_search_enabled']
        self.spiral_initial_radius = np['spiral_initial_radius']
        self.spiral_max_radius = np['spiral_max_radius']
        self.spiral_growth_rate = np['spiral_growth_rate']
        self.spiral_linear_speed = np['spiral_linear_speed']
        self.spiral_angular_speed = np['spiral_angular_speed']
        self.spiral_timeout = np['spiral_timeout']

        # Arrival confirmation (temporal filter)
        self._arrival_first_seen = 0.0   # timestamp when first "close" reading
        self._arrival_confirm_time = 0.3  # seconds of sustained closeness required
        self._arrival_hysteresis = 1.5    # reset timer when distance > threshold * this

        # Close approach distance plateau detector
        self._close_approach_min_dist = float('inf')  # best distance seen
        self._close_approach_plateau_start = 0.0       # when distance stopped decreasing

    def _base_to_laser_angle(self, base_angle):
        """Convert base_link angle to laser frame angle for raw scan lookup.

        The raw scan's angles are in laser frame. When lidar_yaw != 0,
        base_link angles don't map directly to laser scan indices.
        """
        lidar_yaw = math.radians(self._state.detection.get_lidar_yaw())
        return base_angle - lidar_yaw

    def _publish_cmd(self, cmd):
        """Sanitize and publish a Twist command. All publishes go through here."""
        if not _clamp_twist(cmd):
            self._logger.warn("NaN/Inf in Twist command - clamped to zero")
        _cap_speed(cmd, self.linear_speed, self.angular_speed)
        self._cmd_vel_pub.publish(cmd)
        self._state.nav.set_current_cmd_vel(cmd.linear.x, cmd.linear.y, cmd.angular.z)

    # RPLiDAR A1 minimum detection range (~15cm). Below this distance
    # the sensor returns no data, so LiDAR-based signals are unavailable.
    LIDAR_MIN_RANGE = 0.15

    def _check_arrival(self, distance, angle, bbox_clipped=False):
        """Multi-signal arrival check with temporal confirmation.

        Combines up to four independent proximity signals:
            1. Fused distance (vision or LiDAR) below threshold.
            2. LiDAR frontal corridor -- any obstacle directly ahead.
            3. LiDAR at the target angle -- narrow cone toward the target.
            4. Bbox clipped -- target fills camera vertically (very close).

        When the approach threshold is below the LiDAR minimum range
        (~0.15 m for RPLiDAR A1), LiDAR signals are excluded and only
        vision + bbox_clipped are used.  Otherwise 2-of-4 agreement is
        required.  The timer must be sustained for ``_arrival_confirm_time``
        seconds before arrival is declared.

        Args:
            distance: Fused distance to the target [m].
            angle: Target bearing in base_link frame [rad].
            bbox_clipped: True if the detection bounding box is clipped by
                the image edge (vision distance may be unreliable).

        Returns:
            True if arrival is confirmed, False otherwise.
        """
        now = time.time()
        threshold = self.approach_distance

        # Signal 1: fused distance below threshold
        dist_close = distance <= threshold

        # Signal 4: bbox clipped = target fills camera (strong close signal)
        bbox_close = bbox_clipped

        # When threshold is below LiDAR min range, LiDAR signals are useless
        # (sensor physically can't detect at this distance).  Use vision only.
        if threshold < self.LIDAR_MIN_RANGE:
            # Accept arrival on vision distance OR bbox clipped
            is_close = dist_close or bbox_close
            min_front = float('inf')
            lidar_at_target = float('inf')
        else:
            # Signal 2: LiDAR frontal corridor
            min_front, _, _ = self.check_obstacles()
            lidar_front_close = min_front < threshold

            # Signal 3: LiDAR at target angle
            lidar_at_target = self._state.sensors.get_lidar_distance_at_angle(
                self._base_to_laser_angle(angle))
            lidar_target_close = lidar_at_target < threshold

            # Require at least 2 of 4 signals
            close_count = sum([dist_close, lidar_front_close,
                               lidar_target_close, bbox_close])
            is_close = close_count >= 2

        if is_close:
            if self._arrival_first_seen == 0.0:
                self._arrival_first_seen = now
                self._logger.info(
                    f"Arrival timer started: dist={distance:.2f}m, "
                    f"front={min_front:.2f}m, clipped={bbox_clipped}")
            elif now - self._arrival_first_seen >= self._arrival_confirm_time:
                self._logger.info(
                    f"ARRIVED (confirmed {self._arrival_confirm_time:.1f}s): "
                    f"dist={distance:.2f}m, clipped={bbox_clipped}")
                return True
            # Timer running but not yet confirmed
            return False
        else:
            # Reset only beyond hysteresis band
            if distance > threshold * self._arrival_hysteresis:
                self._arrival_first_seen = 0.0
            return False

    def _arrive(self):
        """Execute arrival: stop robot, update state, reset internals."""
        cmd = Twist()
        self._publish_cmd(cmd)
        self._state.nav.set_nav_state("ARRIVED")
        self._state.nav.set_obstacle(False, float('inf'))
        self._arrival_first_seen = 0.0

    def navigate_to_target(self, target):
        """Navigate toward a detected target with LiDAR+Vision distance fusion."""
        vision_distance = target.get('distance', 1.0)
        cam_angle = target.get('angle', 0.0)
        cam_angle = -cam_angle  # Camera convention (+=right) to ROS (+=left)

        # Convert camera-frame angle to robot-frame (base_link) angle
        camera_yaw_rad = math.radians(self._state.detection.get_camera_yaw())
        angle = cam_angle + camera_yaw_rad  # angle is now in base_link frame
        self.last_target_angle = angle
        bbox_clipped = target.get('bbox_clipped', False)

        # Sensor offset compensation: convert distances to active reference point
        # (configurable in Setup tab: center, front, camera, or lidar)
        cam_offset = self._state.detection.get_sensor_offset('camera')
        lidar_offset = self._state.detection.get_sensor_offset('lidar')

        # Vision distance is from camera → apply offset to reference point
        vision_distance_ref = vision_distance + cam_offset

        # Distance fusion: prefer calibrated vision, use LiDAR only when
        # vision is unreliable (bbox clipped = target too close for accurate bbox)
        if bbox_clipped:
            # Convert base_link angle to laser frame for raw scan lookup
            lidar_distance = self._state.sensors.get_lidar_distance_at_angle(
                self._base_to_laser_angle(angle))
            if lidar_distance < float('inf'):
                # LiDAR distance from laser frame → apply offset to reference point
                distance = lidar_distance + lidar_offset
                source = "lidar"
            else:
                distance = vision_distance_ref
                source = "vision"
        else:
            distance = vision_distance_ref
            source = "vision"

        # Reject invalid distances
        if not math.isfinite(distance) or distance <= 0:
            return

        target_x = distance * math.cos(angle)
        target_y = distance * math.sin(angle)

        prev_state, _ = self._state.nav.get_nav_state()
        self._state.nav.set_nav_state("NAVIGATING", (target_x, target_y, distance))

        if prev_state != "NAVIGATING" or time.time() - self._last_nav_log > 2.0:
            clip_tag = " [CLIPPED]" if bbox_clipped else ""
            self._logger.info(
                f"Target at {distance:.2f}m ({source}), "
                f"{math.degrees(angle):.1f}deg{clip_tag}")
            self._last_nav_log = time.time()

        # Store map-frame target for GUI overlay using TF2
        # Transforms detection point: camera_link -> map (via base_link)
        try:
            pt = PointStamped()
            pt.header.stamp = rclpy.time.Time().to_msg()
            pt.header.frame_id = 'camera_link'
            # Detection in camera_link frame (x=forward, y=left)
            # Use cam_angle (camera-frame) — TF2 handles camera_yaw via TF chain
            pt.point.x = vision_distance * math.cos(cam_angle)
            pt.point.y = vision_distance * math.sin(cam_angle)
            pt.point.z = 0.0
            # TF2 handles camera position, yaw, and robot pose automatically
            from tf2_geometry_msgs import do_transform_point
            tf = self._tf_buffer.lookup_transform('map', 'camera_link',
                                                   rclpy.time.Time())
            map_pt = do_transform_point(pt, tf)
            self._state.nav.set_nav_target_map(map_pt.point.x, map_pt.point.y)
        except Exception:
            # Fallback: manual computation if TF not available
            try:
                robot_x, robot_y, robot_theta = self._state.sensors.get_robot_pose()
                cam_pos = self._state.detection.get_camera_pos()
                # Detection in camera frame
                det_x = vision_distance * math.cos(cam_angle)
                det_y = vision_distance * math.sin(cam_angle)
                # Rotate from camera frame to body frame by camera_yaw
                cos_cy = math.cos(camera_yaw_rad)
                sin_cy = math.sin(camera_yaw_rad)
                body_x = cam_pos['x'] + det_x * cos_cy - det_y * sin_cy
                body_y = cam_pos['y'] + det_x * sin_cy + det_y * cos_cy
                cos_t = math.cos(robot_theta)
                sin_t = math.sin(robot_theta)
                map_tx = robot_x + body_x * cos_t - body_y * sin_t
                map_ty = robot_y + body_x * sin_t + body_y * cos_t
                self._state.nav.set_nav_target_map(map_tx, map_ty)
            except Exception:
                pass

        # --- Arrival check (multi-signal with temporal confirmation) ---
        if self._check_arrival(distance, angle, bbox_clipped):
            self._arrive()
            return

        # --- Close approach: target visible and within obstacle avoidance zone ---
        # When the target is close, LiDAR sees it as an obstacle. Instead of
        # triggering VFH avoidance (which backs up or strafes away), drive
        # directly toward it at slow speed.  No plateau detector — just keep
        # driving until arrival check fires or blind approach takes over.
        if distance < self.obstacle_slowdown_distance:
            cmd = Twist()
            speed = self.min_linear_speed
            cmd.linear.x = speed * math.cos(angle)
            cmd.linear.y = speed * math.sin(angle)
            cmd.angular.z = max(-0.15, min(0.15, angle * 0.3))
            self._publish_cmd(cmd)
            return

        # --- Bbox clipped + vision-only: vision distance is unreliable ---
        # Clipped bbox means target is close but measured height is too small,
        # so vision overestimates distance. Enter blind approach rather than
        # navigating with wrong data.
        if bbox_clipped and source == "vision" and distance < self.blind_approach_entry_distance:
            self._logger.info(
                f"Bbox clipped + vision-only at {distance:.2f}m -> early blind approach")
            if self.enter_blind_approach():
                self.blind_approach_step()
                return

        # --- During arrival confirmation: slow creep instead of full nav ---
        if self._arrival_first_seen != 0.0:
            cmd = Twist()
            creep_speed = self.min_linear_speed * 0.5
            cmd.linear.x = creep_speed * math.cos(angle)
            cmd.linear.y = creep_speed * math.sin(angle)
            cmd.angular.z = max(-0.15, min(0.15, angle * 0.3))
            self._publish_cmd(cmd)
            return

        self.direct_navigate(target_x, target_y, distance, target_visible=True)

    def direct_navigate(self, target_x, target_y, distance, target_visible=False):
        """Mecanum holonomic navigation with VFH obstacle avoidance.

        Uses Vector Field Histogram (VFH) gap-finding to steer around obstacles.
        When *target_visible* is True, keeps the camera pointed at the target
        while the body strafes through gaps (LiDAR-gated visual servoing).

        Args:
            target_x: Target X position in robot (base_link) frame [m].
            target_y: Target Y position in robot (base_link) frame [m].
            distance: Euclidean distance to the target [m].
            target_visible: If True, fuse angular control to keep the camera
                aimed at the target during obstacle avoidance.
        """
        angle = math.atan2(target_y, target_x)
        cmd = Twist()
        now = time.time()

        min_front, left_free, right_free = self.check_obstacles()
        self._state.nav.set_obstacle(min_front < self.obstacle_distance, min_front)

        # When target is visible and the closest obstacle is roughly in the
        # target direction, the obstacle IS the target (bottle). Skip VFH
        # avoidance and drive straight at it (close approach).
        if target_visible and min_front < self.obstacle_distance:
            lidar_at_target = self._state.sensors.get_lidar_distance_at_angle(
                self._base_to_laser_angle(angle))
            # If LiDAR at target angle is close, obstacle = target → approach
            if lidar_at_target < self.obstacle_slowdown_distance:
                speed = self.min_linear_speed
                cmd.linear.x = speed * math.cos(angle)
                cmd.linear.y = speed * math.sin(angle)
                cmd.angular.z = max(-0.15, min(0.15, angle * 0.3))
                self._publish_cmd(cmd)
                return

        if min_front < self.obstacle_distance:
            # --- OBSTACLE: VFH avoidance ---
            if min_front < self.LIDAR_MIN_RANGE:
                # Emergency: too close — back up immediately + strafe to clear
                cmd.linear.x = -self.linear_speed * 0.4
                if left_free:
                    cmd.linear.y = self.min_linear_speed
                elif right_free:
                    cmd.linear.y = -self.min_linear_speed
            else:
                self._stuck_start = 0.0
                gap_angle = self._find_best_gap(angle)

                if gap_angle is not None:
                    # Commit to avoidance direction (prevents oscillation)
                    gap_side = 1 if gap_angle > 0 else -1
                    if self._avoid_direction == 0:
                        self._avoid_direction = gap_side
                        self._avoid_start_time = now
                    elif now - self._avoid_start_time > self._avoid_min_commit:
                        self._avoid_direction = gap_side

                    if self._avoid_direction != gap_side:
                        committed_gap = self._find_committed_gap(angle)
                        if committed_gap is not None:
                            gap_angle = committed_gap

                    # Drive through the gap using mecanum strafing
                    speed = self.linear_speed * 0.6
                    cmd.linear.x = speed * math.cos(gap_angle)
                    cmd.linear.y = speed * math.sin(gap_angle)

                    if target_visible:
                        # FUSED: keep camera on target while body strafes
                        target_ang = self.last_target_angle
                        cmd.angular.z = max(-self.angular_speed * 0.4,
                                            min(self.angular_speed * 0.4,
                                                target_ang * 0.5))
                    else:
                        # No target: rotate toward gap (original behavior)
                        cmd.angular.z = max(-self.angular_speed * 0.3,
                                            min(self.angular_speed * 0.3,
                                                gap_angle * 0.3))
                else:
                    # No gap: back up (with stuck timeout)
                    cmd.linear.x = -self.linear_speed * 0.3
                    self._avoid_direction = 0
                    if self._stuck_start == 0.0:
                        self._stuck_start = now
                    elif now - self._stuck_start > 3.0:
                        # Stuck backing up for 3s with no gap — stop
                        self._logger.warn("VFH: stuck backing up >3s, stopping")
                        self.stop_robot()
                        return

            if now - self._last_obs_log > 1.0:
                fused = " [FUSED]" if target_visible else ""
                self._logger.warn(
                    f"OBSTACLE at {min_front:.2f}m! "
                    f"avoid={'L' if self._avoid_direction > 0 else 'R' if self._avoid_direction < 0 else '-'}"
                    f"{fused}")
                self._last_obs_log = now
        else:
            # --- CLEAR PATH ---
            # Only reset avoidance after sustained clearance (5 ticks ~0.5s)
            if self._avoid_direction != 0:
                self._avoid_clear_count += 1
                if self._avoid_clear_count >= 5:
                    self._avoid_direction = 0
                    self._avoid_clear_count = 0
            else:
                self._avoid_clear_count = 0

            self._stuck_start = 0.0
            speed_scale = min(1.0, distance / 0.40)
            speed = self.linear_speed * max(0.4, speed_scale)
            cmd.linear.x = speed * math.cos(angle)
            cmd.linear.y = speed * math.sin(angle)
            cmd.angular.z = max(-self.angular_speed * 0.4,
                                min(self.angular_speed * 0.4, angle * 0.5))

            if min_front < self.obstacle_slowdown_distance:
                denom = self.obstacle_slowdown_distance - self.obstacle_distance
                if denom > 0:
                    slow_factor = (min_front - self.obstacle_distance) / denom
                    slow = max(0.3, min(1.0, slow_factor))
                    cmd.linear.x *= slow
                    cmd.linear.y *= slow

            # Enforce minimum speed (motor dead zone)
            total_speed = math.sqrt(cmd.linear.x ** 2 + cmd.linear.y ** 2)
            if total_speed > 0 and total_speed < self.min_linear_speed:
                scale_up = self.min_linear_speed / total_speed
                cmd.linear.x *= scale_up
                cmd.linear.y *= scale_up

        # Side obstacle protection: don't strafe into walls
        if not left_free and cmd.linear.y > 0:
            cmd.linear.y = 0.0
        if not right_free and cmd.linear.y < 0:
            cmd.linear.y = 0.0

        self._publish_cmd(cmd)

    def _find_committed_gap(self, target_angle):
        """Find the best gap restricted to the committed avoidance side.

        Same algorithm as ``_find_best_gap`` but only considers gaps on
        the side determined by ``_avoid_direction`` (left or right).
        This prevents oscillation when the robot has already committed
        to a particular avoidance direction.

        Args:
            target_angle: Desired travel direction in robot frame [rad].

        Returns:
            Gap angle [rad] on the committed side, or None.
        """
        points, _, _ = self._state.sensors.get_laser()
        if len(points) == 0:
            return None

        num_bins = 36
        bin_size = math.pi / num_bins
        histogram = [float('inf')] * num_bins

        for x, y in points:
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            dist = math.sqrt(x * x + y * y)
            if dist < 0.03:
                continue
            ang = math.atan2(y, x)
            if abs(ang) > math.pi / 2:
                continue
            idx = int((ang + math.pi / 2) / bin_size)
            idx = max(0, min(num_bins - 1, idx))
            histogram[idx] = min(histogram[idx], dist)

        clear_dist = self.obstacle_distance + 0.10
        body_angle = 2.0 * math.atan2(self.robot_half_width + 0.03, clear_dist)
        body_bins = max(1, int(math.ceil(body_angle / bin_size)))
        half_body = body_bins // 2

        best_angle = None
        best_cost = float('inf')

        for i in range(num_bins):
            gap_angle = -math.pi / 2 + (i + 0.5) * bin_size
            # Only check committed side
            if self._avoid_direction > 0 and gap_angle < 0:
                continue
            if self._avoid_direction < 0 and gap_angle > 0:
                continue
            lo = max(0, i - half_body)
            hi = min(num_bins, i + half_body + 1)
            if all(histogram[j] > clear_dist for j in range(lo, hi)):
                cost = abs(math.atan2(
                    math.sin(gap_angle - target_angle),
                    math.cos(gap_angle - target_angle)))
                if cost < best_cost:
                    best_cost = cost
                    best_angle = gap_angle

        return best_angle

    def enter_blind_approach(self):
        """Enter blind approach mode: dead-reckon to last known target position.

        Converts the last base_link-frame target to map coordinates using
        the current odometry pose, then stores it as the blind approach goal.

        Returns:
            True if blind approach was successfully entered, False if the
            robot pose is stale (default 0,0,0) or no navigation target
            is available.
        """
        robot_x, robot_y, robot_theta = self._state.sensors.get_robot_pose()

        # Reject if pose looks like default (0,0,0) - likely stale/unavailable
        if robot_x == 0.0 and robot_y == 0.0 and robot_theta == 0.0:
            self._logger.warn("BLIND_APPROACH: rejected - robot pose is default (0,0,0)")
            return False

        _, nav_target = self._state.nav.get_nav_state()
        if nav_target is None:
            return False

        target_x_bl, target_y_bl, distance = nav_target
        cos_t = math.cos(robot_theta)
        sin_t = math.sin(robot_theta)
        target_x_map = robot_x + target_x_bl * cos_t - target_y_bl * sin_t
        target_y_map = robot_y + target_x_bl * sin_t + target_y_bl * cos_t

        self.blind_approach_active = True
        self.blind_approach_start_time = time.time()
        self.blind_approach_start_pose = (robot_x, robot_y, robot_theta)
        self.blind_approach_target_map = (target_x_map, target_y_map)
        self.blind_approach_target_distance = distance

        self._state.nav.set_nav_state("BLIND_APPROACH", nav_target)
        self._logger.info(
            f"BLIND_APPROACH: target_map=({target_x_map:.2f}, {target_y_map:.2f}), "
            f"distance={distance:.2f}m")
        return True

    def blind_approach_step(self):
        """Execute one tick of blind approach dead-reckoning.

        Computes the remaining distance and heading error to the stored
        map-frame target, then drives holonomically toward it.  Exits on
        any of these conditions:
            - Arrived (remaining distance within margin).
            - Timeout exceeded.
            - LiDAR obstacle within stop distance (declared ARRIVED if
              close to target, otherwise aborts to search).
            - Heading diverged more than 45 degrees.
            - Robot pose is stale (0,0,0).
        """
        robot_x, robot_y, robot_theta = self._state.sensors.get_robot_pose()

        # Reject stale pose: if still at default, abort
        if robot_x == 0.0 and robot_y == 0.0 and robot_theta == 0.0:
            self._logger.warn("BLIND_APPROACH: pose is (0,0,0) - aborting")
            self.stop_robot()
            self.blind_approach_active = False
            self._state.nav.start_search()
            return

        target_x, target_y = self.blind_approach_target_map

        dx = target_x - robot_x
        dy = target_y - robot_y
        remaining = math.sqrt(dx * dx + dy * dy)
        desired_heading = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(desired_heading - robot_theta),
            math.cos(desired_heading - robot_theta))
        elapsed = time.time() - self.blind_approach_start_time

        # Exit: arrived
        if remaining <= self.blind_approach_arrival_margin:
            self._logger.info(f"BLIND_APPROACH: Arrived! remaining={remaining:.3f}m")
            self.stop_robot()
            self.blind_approach_active = False
            self._state.nav.set_nav_state("ARRIVED")
            return

        # Exit: timeout
        if elapsed > self.blind_approach_timeout:
            self._logger.warn(f"BLIND_APPROACH: Timeout after {elapsed:.1f}s")
            self.stop_robot()
            self.blind_approach_active = False
            self._state.nav.start_search()
            return

        # Exit: LiDAR obstacle — only declare ARRIVED if remaining distance is also small
        # Use max of configured threshold and LiDAR min range (sensor can't see closer)
        min_front, left_free, right_free = self.check_obstacles()
        lidar_stop = max(self.blind_approach_lidar_stop, self.LIDAR_MIN_RANGE)
        if min_front < lidar_stop:
            if remaining < self.blind_approach_arrival_margin * 4:
                self._logger.info(
                    f"BLIND_APPROACH: LiDAR stop at {min_front:.3f}m, "
                    f"remaining={remaining:.3f}m -> ARRIVED")
                self.stop_robot()
                self.blind_approach_active = False
                self._state.nav.set_nav_state("ARRIVED")
                return
            else:
                # LiDAR obstacle but far from target — likely a wall, abort to search
                self._logger.warn(
                    f"BLIND_APPROACH: LiDAR obstacle at {min_front:.3f}m but "
                    f"remaining={remaining:.2f}m -> search")
                self.stop_robot()
                self.blind_approach_active = False
                self._state.nav.start_search()
                return

        # Exit: heading diverged
        if abs(heading_error) > math.radians(45):
            self._logger.warn(f"BLIND_APPROACH: Heading diverged {math.degrees(heading_error):.1f}deg")
            self.stop_robot()
            self.blind_approach_active = False
            self._state.nav.start_search()
            return

        # Drive with mecanum holonomic motion
        cmd = Twist()
        # Ramp down speed in last 30cm. min_linear_speed is the motor dead zone floor.
        speed_ramp = min(1.0, remaining / 0.30)
        speed = max(self.min_linear_speed, self.blind_approach_speed * speed_ramp)
        cmd.linear.x = speed * math.cos(heading_error)
        cmd.linear.y = speed * math.sin(heading_error)
        cmd.angular.z = max(-0.2, min(0.2, 0.5 * heading_error))

        # Side obstacle protection: suppress lateral velocity into blocked side
        if not left_free and cmd.linear.y > 0:
            cmd.linear.y = 0.0
        if not right_free and cmd.linear.y < 0:
            cmd.linear.y = 0.0
        # Slow down when front obstacle is close
        if min_front < self.obstacle_distance:
            cmd.linear.x *= 0.3

        self._publish_cmd(cmd)

    def start_search_scan(self):
        """Initialize a 360° search rotation."""
        _, _, yaw = self._state.sensors.get_robot_pose()
        self._search_start_yaw = yaw
        self._search_last_yaw = yaw
        self._search_total_rotated = 0.0
        self._search_initialized = True
        self._logger.info(
            f"360° search scan started at yaw={math.degrees(yaw):.1f}°")

    def search_rotate(self):
        """Execute one tick of 360° search scan. Returns True when complete."""
        if not self._search_initialized:
            self.start_search_scan()

        # Track rotation via odometry
        _, _, current_yaw = self._state.sensors.get_robot_pose()
        delta = math.atan2(
            math.sin(current_yaw - self._search_last_yaw),
            math.cos(current_yaw - self._search_last_yaw))
        self._search_total_rotated += abs(delta)
        self._search_last_yaw = current_yaw

        # Check if we've completed 360° (~6.28 radians)
        if self._search_total_rotated >= 2 * math.pi:
            self.stop_robot()
            self._search_initialized = False
            self._logger.info("360° search scan complete - target not found")
            return True  # Search complete, target not found

        # Fixed rotation direction (toward last known target angle)
        # Never reverse — reversing causes oscillation with nearby obstacles
        direction = 1.0 if self.last_target_angle >= 0 else -1.0

        cmd = Twist()
        cmd.angular.z = self.search_angular_speed * direction

        # Potential field: repulsive force from ALL nearby obstacles (360°)
        rep_x, rep_y = self._compute_repulsion(self.obstacle_distance + 0.10)
        rep_mag = math.sqrt(rep_x * rep_x + rep_y * rep_y)
        if rep_mag > 0.01:
            # Normalize and drive at min_linear_speed away from obstacles
            cmd.linear.x = self.min_linear_speed * (rep_x / rep_mag)
            cmd.linear.y = self.min_linear_speed * (rep_y / rep_mag)

        self._publish_cmd(cmd)
        return False  # Still searching

    # ------------------------------------------------------------------
    # Spiral search (Tier 2: expanding spiral after 360° scan fails)
    # ------------------------------------------------------------------

    def start_spiral_search(self):
        """Initialize Tier 2 expanding spiral search from current position."""
        rx, ry, rt = self._state.sensors.get_robot_pose()
        self._spiral_active = True
        self._spiral_start_time = time.time()
        self._spiral_start_x = rx
        self._spiral_start_y = ry
        self._spiral_angle = rt  # Start heading in current direction
        self._spiral_radius = self.spiral_initial_radius
        self._spiral_last_time = time.time()
        self._state.nav.set_nav_state("SPIRAL_SEARCH")
        self._logger.info(
            f"Spiral search started at ({rx:.2f}, {ry:.2f}), "
            f"r={self.spiral_initial_radius:.2f}m")

    def spiral_search_step(self):
        """Execute one tick of expanding Archimedean spiral search (Tier 2).

        The robot follows a spiral path centered on its position when the
        search started, with the radius growing linearly per revolution.
        A sinusoidal angular oscillation sweeps the camera for wider
        detection coverage.  Obstacle avoidance (VFH gap-finding) is active
        during the spiral.

        Returns:
            True when the search is exhausted (timeout or max radius
            reached), False while the spiral is still active.
        """
        if not self._spiral_active:
            return True

        now = time.time()
        elapsed = now - self._spiral_start_time

        if elapsed > self.spiral_timeout:
            self._logger.warn(f"Spiral search timeout after {elapsed:.1f}s")
            self.stop_robot()
            return True

        if self._spiral_radius > self.spiral_max_radius:
            self._logger.warn(
                f"Spiral search max radius {self.spiral_max_radius:.1f}m reached")
            self.stop_robot()
            return True

        rx, ry, rt = self._state.sensors.get_robot_pose()

        # Advance spiral: angular rate = linear_speed / radius
        dt = min(now - self._spiral_last_time, 0.2)  # clamp to prevent jumps
        self._spiral_last_time = now
        if self._spiral_radius > 0.05:
            angular_rate = self.spiral_linear_speed / self._spiral_radius
        else:
            angular_rate = 2.0
        self._spiral_angle += angular_rate * dt

        # Archimedean spiral: radius grows linearly with angle
        self._spiral_radius = (self.spiral_initial_radius +
                               self.spiral_growth_rate *
                               self._spiral_angle / (2 * math.pi))

        # Target point on spiral (map frame, centered on start)
        tx = (self._spiral_start_x +
              self._spiral_radius * math.cos(self._spiral_angle))
        ty = (self._spiral_start_y +
              self._spiral_radius * math.sin(self._spiral_angle))

        # Convert to robot-frame displacement
        dx, dy = tx - rx, ty - ry
        cos_r, sin_r = math.cos(-rt), math.sin(-rt)
        local_x = dx * cos_r - dy * sin_r
        local_y = dx * sin_r + dy * cos_r

        # Obstacle avoidance
        min_front, left_free, right_free = self.check_obstacles()
        cmd = Twist()

        if min_front < self.obstacle_distance:
            move_angle = math.atan2(local_y, local_x)
            gap = self._find_best_gap(move_angle)
            if gap is not None:
                speed = self.spiral_linear_speed * 0.5
                cmd.linear.x = speed * math.cos(gap)
                cmd.linear.y = speed * math.sin(gap)
            else:
                cmd.linear.x = -self.spiral_linear_speed * 0.3
        else:
            dist = math.sqrt(local_x ** 2 + local_y ** 2)
            if dist > 0.02:
                move_angle = math.atan2(local_y, local_x)
                cmd.linear.x = self.spiral_linear_speed * math.cos(move_angle)
                cmd.linear.y = self.spiral_linear_speed * math.sin(move_angle)
            else:
                cmd.linear.x = self.spiral_linear_speed * 0.5

        # Side obstacle protection
        if not left_free and cmd.linear.y > 0:
            cmd.linear.y = 0.0
        if not right_free and cmd.linear.y < 0:
            cmd.linear.y = 0.0

        # Camera sweep: sinusoidal oscillation for ~±17° coverage
        sweep_phase = elapsed * 0.8
        cmd.angular.z = self.spiral_angular_speed * math.sin(sweep_phase)

        # Enforce minimum speed (motor dead zone)
        total_speed = math.sqrt(cmd.linear.x ** 2 + cmd.linear.y ** 2)
        if total_speed > 0 and total_speed < self.min_linear_speed:
            scale = self.min_linear_speed / total_speed
            cmd.linear.x *= scale
            cmd.linear.y *= scale

        self._publish_cmd(cmd)
        return False  # Still searching

    def stop_robot(self):
        """Stop all robot motion."""
        cmd = Twist()
        self._publish_cmd(cmd)
        self._stuck_start = 0.0
        self._arrival_first_seen = 0.0
        self._avoid_direction = 0
        self._avoid_clear_count = 0
        self._search_initialized = False
        self._spiral_active = False
        self._close_approach_min_dist = float('inf')
        self._close_approach_plateau_start = 0.0

    def _find_best_gap(self, target_angle):
        """VFH-lite: find the best open direction toward the target.

        Builds a polar histogram (front 180 deg, 36 bins of 5 deg each) from
        LiDAR points.  Each bin records the closest obstacle distance.  A gap
        is a contiguous set of bins wide enough to fit the robot body
        (``robot_half_width`` + margin).  The gap whose center is closest to
        *target_angle* is selected.

        Args:
            target_angle: Desired travel direction in robot frame [rad].

        Returns:
            Steering angle in robot frame [rad], or None if no passable gap
            exists in the front hemisphere.
        """
        points, _, laser_time = self._state.sensors.get_laser()
        if len(points) == 0:
            return None

        # Polar histogram: front 180° in 5° bins
        num_bins = 36
        bin_size = math.pi / num_bins
        histogram = [float('inf')] * num_bins

        for x, y in points:
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            dist = math.sqrt(x * x + y * y)
            if dist < 0.03:
                continue
            ang = math.atan2(y, x)
            if abs(ang) > math.pi / 2:
                continue  # behind robot
            idx = int((ang + math.pi / 2) / bin_size)
            idx = max(0, min(num_bins - 1, idx))
            histogram[idx] = min(histogram[idx], dist)

        # How many bins the robot body spans at the clearance distance
        clear_dist = self.obstacle_distance + 0.10
        body_angle = 2.0 * math.atan2(self.robot_half_width + 0.03, clear_dist)
        body_bins = max(1, int(math.ceil(body_angle / bin_size)))
        half_body = body_bins // 2

        # Score each direction: must be clear for robot width, prefer closer to target
        best_angle = None
        best_cost = float('inf')

        for i in range(num_bins):
            lo = max(0, i - half_body)
            hi = min(num_bins, i + half_body + 1)
            if all(histogram[j] > clear_dist for j in range(lo, hi)):
                gap_angle = -math.pi / 2 + (i + 0.5) * bin_size
                cost = abs(math.atan2(
                    math.sin(gap_angle - target_angle),
                    math.cos(gap_angle - target_angle)))
                if cost < best_cost:
                    best_cost = cost
                    best_angle = gap_angle

        return best_angle

    def _compute_repulsion(self, influence_dist):
        """Potential field: compute repulsive vector from ALL nearby LiDAR points (360°).

        Returns (rx, ry) in robot frame — the direction to escape obstacles.
        Magnitude indicates urgency. Returns (0, 0) if no nearby obstacles.
        """
        points, _, laser_time = self._state.sensors.get_laser()
        if len(points) == 0:
            return 0.0, 0.0

        rx, ry = 0.0, 0.0
        inv_inf2 = 1.0 / (influence_dist * influence_dist)

        for x, y in points:
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            d2 = x * x + y * y
            dist = math.sqrt(d2)
            if dist < 0.03 or dist > influence_dist:
                continue
            # Repulsive force ∝ (1/d² - 1/d_inf²), direction = away from obstacle
            force = (1.0 / d2) - inv_inf2
            rx -= (x / dist) * force
            ry -= (y / dist) * force

        return rx, ry

    def check_obstacles(self):
        """Check LiDAR for obstacles. Returns (min_front_dist, left_free, right_free).

        Fail-safe: returns (0.0, False, False) if LiDAR data is stale or empty,
        which triggers emergency stop rather than driving blind.
        """
        points, _, laser_time = self._state.sensors.get_laser()

        # Fail-safe: stale or missing LiDAR data → assume obstacle everywhere
        # Use 2.0s threshold: RPLiDAR A1 runs at 5-10Hz, plus IPC sync adds
        # latency.  0.5s was too tight and falsely triggered during normal
        # operation, causing the robot to stop at 0.78m.
        if len(points) == 0 or (laser_time > 0 and time.time() - laser_time > 2.0):
            if time.time() - self._last_obs_log > 2.0:
                self._logger.warn("LiDAR data stale or empty - fail-safe stop")
                self._last_obs_log = time.time()
            return 0.0, False, False

        hw = self.robot_half_width
        # Wider front corridor: use hw + margin to catch angled approaches
        front_hw = hw + 0.05
        slow_dist = self.obstacle_slowdown_distance
        min_front = float('inf')
        left_free = True
        right_free = True

        for x, y in points:
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            # Front corridor: only forward points
            if 0 < x <= slow_dist and abs(y) < front_hw:
                min_front = min(min_front, x)
            # Side checks: include points beside AND slightly behind the robot
            # (mecanum can strafe, so side/rear obstacles matter)
            if -hw < x < slow_dist and hw <= y < hw + 0.4:
                left_free = False
            if -hw < x < slow_dist and -(hw + 0.4) < y <= -hw:
                right_free = False

        return min_front, left_free, right_free
