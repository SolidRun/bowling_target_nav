"""Mecanum holonomic navigation with obstacle avoidance and blind approach."""

import math
import time

from geometry_msgs.msg import Twist


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

        # Navigation parameters (tunable via SettingsWindow)
        self.approach_distance = 0.15
        self.linear_speed = 0.15
        self.min_linear_speed = 0.10
        self.angular_speed = 0.5
        self.obstacle_distance = 0.25
        self.obstacle_slowdown_distance = 0.5
        self.robot_half_width = 0.18
        self.search_angular_speed = 0.4
        self.lost_timeout = 3.0
        self.search_timeout = 30.0

        # Blind approach parameters
        self.blind_approach_active = False
        self.blind_approach_start_time = 0.0
        self.blind_approach_start_pose = (0.0, 0.0, 0.0)
        self.blind_approach_target_map = (0.0, 0.0)
        self.blind_approach_target_distance = 0.0
        self.blind_approach_entry_distance = 0.80
        self.blind_approach_speed = 0.10
        self.blind_approach_timeout = 8.0
        self.blind_approach_lidar_stop = 0.12
        self.blind_approach_arrival_margin = 0.10

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

        # 360° search scan state
        self._search_start_yaw = 0.0
        self._search_last_yaw = 0.0
        self._search_total_rotated = 0.0
        self._search_initialized = False

        # Arrival confirmation (temporal filter)
        self._arrival_first_seen = 0.0   # timestamp when first "close" reading
        self._arrival_confirm_time = 0.3  # seconds of sustained closeness required
        self._arrival_hysteresis = 1.5    # reset timer when distance > threshold * this

    def _publish_cmd(self, cmd):
        """Sanitize and publish a Twist command. All publishes go through here."""
        if not _clamp_twist(cmd):
            self._logger.warn("NaN/Inf in Twist command - clamped to zero")
        _cap_speed(cmd, self.linear_speed, self.angular_speed)
        self._cmd_vel_pub.publish(cmd)
        self._state.nav.set_current_cmd_vel(cmd.linear.x, cmd.linear.y, cmd.angular.z)

    def _check_arrival(self, distance, angle, bbox_clipped=False):
        """Multi-signal arrival check with temporal confirmation.

        Combines fused distance, LiDAR frontal corridor, and LiDAR at target
        angle. Requires sustained close readings for 0.3s to confirm arrival.

        Returns True if arrived, False otherwise.
        """
        now = time.time()
        threshold = self.approach_distance

        # Signal 1: fused distance below threshold
        dist_close = distance <= threshold

        # Signal 2: LiDAR frontal corridor (any obstacle directly ahead)
        min_front, _, _ = self.check_obstacles()
        lidar_front_close = min_front < threshold

        # Signal 3: LiDAR at target angle (narrow cone toward the pin)
        lidar_at_target = self._state.sensors.get_lidar_distance_at_angle(angle)
        lidar_target_close = lidar_at_target < threshold

        # Emergency override: something very close in front + evidence of target
        if min_front < self.blind_approach_lidar_stop and (dist_close or bbox_clipped):
            self._logger.info(
                f"ARRIVED (emergency): front={min_front:.3f}m, "
                f"dist={distance:.2f}m, clipped={bbox_clipped}")
            return True

        # Any signal starts the confirmation timer
        is_close = dist_close or lidar_front_close or lidar_target_close

        if is_close:
            if self._arrival_first_seen == 0.0:
                self._arrival_first_seen = now
                self._logger.info(
                    f"Arrival timer started: dist={distance:.2f}m, "
                    f"front={min_front:.2f}m, lidar_target={lidar_at_target:.2f}m")
            elif now - self._arrival_first_seen >= self._arrival_confirm_time:
                self._logger.info(
                    f"ARRIVED (confirmed {self._arrival_confirm_time:.1f}s): "
                    f"dist={distance:.2f}m, front={min_front:.2f}m")
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
        angle = target.get('angle', 0.0)
        angle = -angle  # Camera convention (+=right) to ROS (+=left)
        self.last_target_angle = angle
        bbox_clipped = target.get('bbox_clipped', False)

        # LiDAR+Vision fusion
        lidar_distance = self._state.sensors.get_lidar_distance_at_angle(angle)
        if lidar_distance < float('inf'):
            distance = lidar_distance
            source = "lidar"
        else:
            distance = vision_distance
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

        # Store map-frame target for GUI overlay
        try:
            robot_x, robot_y, robot_theta = self._state.sensors.get_robot_pose()
            cos_t = math.cos(robot_theta)
            sin_t = math.sin(robot_theta)
            map_tx = robot_x + target_x * cos_t - target_y * sin_t
            map_ty = robot_y + target_x * sin_t + target_y * cos_t
            self._state.nav.set_nav_target_map(map_tx, map_ty)
        except Exception:
            pass

        # --- Arrival check (multi-signal with temporal confirmation) ---
        if self._check_arrival(distance, angle, bbox_clipped):
            self._arrive()
            return

        # --- Bbox clipped + vision-only: vision distance is unreliable ---
        # Clipped bbox means pin is close but measured height is too small,
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

        self.direct_navigate(target_x, target_y, distance)

    def direct_navigate(self, target_x, target_y, distance):
        """Mecanum holonomic navigation with VFH obstacle avoidance.

        Uses Vector Field Histogram (VFH) gap-finding to steer around obstacles
        instead of simple left/right strafing. Commits to a direction to prevent
        oscillation.
        """
        angle = math.atan2(target_y, target_x)
        cmd = Twist()
        now = time.time()

        min_front, left_free, right_free = self.check_obstacles()
        self._state.nav.set_obstacle(min_front < self.obstacle_distance, min_front)

        if min_front < self.obstacle_distance:
            # --- OBSTACLE: VFH avoidance ---
            if min_front < self.approach_distance:
                # Emergency: too close
                if self._stuck_start == 0.0:
                    self._stuck_start = now
                elif now - self._stuck_start > 3.0:
                    cmd.linear.x = -self.linear_speed * 0.3
                    self._stuck_start = 0.0
            else:
                self._stuck_start = 0.0
                gap_angle = self._find_best_gap(angle)

                if gap_angle is not None:
                    # Commit to avoidance direction (prevents oscillation)
                    gap_side = 1 if gap_angle > 0 else -1
                    if self._avoid_direction == 0:
                        # First entry: commit
                        self._avoid_direction = gap_side
                        self._avoid_start_time = now
                    elif now - self._avoid_start_time > self._avoid_min_commit:
                        # Commitment expired: allow switching
                        self._avoid_direction = gap_side

                    # If committed direction still has a gap, prefer it
                    # Otherwise follow best gap regardless
                    if self._avoid_direction != gap_side:
                        # Check if committed side has any valid gap
                        committed_gap = self._find_committed_gap(angle)
                        if committed_gap is not None:
                            gap_angle = committed_gap

                    # Drive through the gap
                    speed = self.linear_speed * 0.6
                    cmd.linear.x = speed * math.cos(gap_angle)
                    cmd.linear.y = speed * math.sin(gap_angle)
                    cmd.angular.z = max(-self.angular_speed * 0.3,
                                        min(self.angular_speed * 0.3,
                                            gap_angle * 0.3))
                else:
                    # No gap: back up
                    cmd.linear.x = -self.linear_speed * 0.3
                    self._avoid_direction = 0

            if now - self._last_obs_log > 1.0:
                self._logger.warn(
                    f"OBSTACLE at {min_front:.2f}m! "
                    f"avoid={'L' if self._avoid_direction > 0 else 'R' if self._avoid_direction < 0 else '-'}")
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

        self._publish_cmd(cmd)

    def _find_committed_gap(self, target_angle):
        """Find the best gap on the committed avoidance side."""
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
        """Enter blind approach mode: dead-reckon to last known pin position."""
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
        """Execute one tick of blind approach dead-reckoning."""
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

        # Exit: LiDAR obstacle
        min_front, left_free, right_free = self.check_obstacles()
        if min_front < self.blind_approach_lidar_stop:
            self._logger.info(f"BLIND_APPROACH: LiDAR stop at {min_front:.3f}m")
            self.stop_robot()
            self.blind_approach_active = False
            self._state.nav.set_nav_state("ARRIVED")
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

        # Determine rotation direction (toward last known target angle)
        direction = 1.0 if self.last_target_angle >= 0 else -1.0

        # Obstacle avoidance during search
        min_front, left_free, right_free = self.check_obstacles()
        if min_front < self.obstacle_distance:
            if direction > 0 and not left_free:
                direction = -1.0
            elif direction < 0 and not right_free:
                direction = 1.0

        cmd = Twist()
        cmd.angular.z = self.search_angular_speed * direction
        self._publish_cmd(cmd)
        return False  # Still searching

    def stop_robot(self):
        """Stop all robot motion."""
        cmd = Twist()
        self._cmd_vel_pub.publish(cmd)
        self._state.nav.set_current_cmd_vel(0.0, 0.0, 0.0)
        self._stuck_start = 0.0
        self._arrival_first_seen = 0.0
        self._avoid_direction = 0
        self._avoid_clear_count = 0
        self._search_initialized = False

    def _find_best_gap(self, target_angle):
        """VFH-lite: find the best open direction toward the target.

        Builds a polar histogram from LiDAR, finds gaps wide enough for the
        robot body, and returns the gap direction closest to target_angle.

        Returns steering angle in robot frame, or None if no gap found.
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

    def check_obstacles(self):
        """Check LiDAR for obstacles. Returns (min_front_dist, left_free, right_free).

        Fail-safe: returns (0.0, False, False) if LiDAR data is stale or empty,
        which triggers emergency stop rather than driving blind.
        """
        points, _, laser_time = self._state.sensors.get_laser()

        # Fail-safe: stale or missing LiDAR data → assume obstacle everywhere
        if len(points) == 0 or (laser_time > 0 and time.time() - laser_time > 0.5):
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
            if x <= 0 or x > slow_dist:
                continue
            if abs(y) < front_hw:
                min_front = min(min_front, x)
            if x < slow_dist and hw <= y < hw + 0.4:
                left_free = False
            if x < slow_dist and -(hw + 0.4) < y <= -hw:
                right_free = False

        return min_front, left_free, right_free
