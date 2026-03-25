"""VFH (Vector Field Histogram) obstacle avoidance mixin for Navigator.

Provides ObstacleAvoidanceMixin, which implements holonomic obstacle avoidance
for a mecanum-drive robot using a simplified VFH algorithm. The mixin is
composed into the Navigator class via multiple inheritance.

VFH Algorithm Overview:
    VFH divides the robot's surroundings into angular sectors and builds a
    polar histogram of obstacle distances. The robot then finds the widest
    "gap" (obstacle-free sector) closest to its desired heading and steers
    through it.

    Step 1 — Polar histogram:
        The front 180° is divided into VFH_NUM_BINS (36) sectors of 10° each.
        For each LiDAR point, the nearest-obstacle distance is recorded in
        the corresponding sector bin.

    Step 2 — Gap finding:
        A "gap" is a contiguous run of bins where ALL obstacles are farther
        than obstacle_distance + VFH_CLEARANCE_MARGIN (0.10m) and the gap
        is wide enough for the robot body (robot_half_width + VFH_BODY_MARGIN).
        The gap whose center is closest to the target bearing is chosen.

    Step 3 — Direction commitment:
        Once a gap direction (left or right) is chosen, the robot commits to
        it for VFH_AVOID_COMMIT_SECS (1.5s) to prevent oscillation between
        nearby gaps.

    Step 4 — Mecanum strafing:
        Unlike differential-drive VFH, the mecanum robot can strafe through
        the gap while independently rotating to keep the camera on target
        (fused mode). This allows obstacle avoidance without losing sight
        of the target.

    Step 5 — Emergency backup:
        If no passable gap exists, the robot reverses slowly. After
        VFH_STUCK_TIMEOUT_SECS (3.0s) of backing up, a full stop is triggered.

Key methods:
  - direct_navigate(): Main entry point -- drives toward a target with VFH
    gap-finding when obstacles are detected in the front corridor.
  - check_obstacles(): Scans LiDAR points for front/side obstacles with
    fail-safe behavior on stale data.
  - _find_best_gap(): Builds a polar histogram and finds the widest passable
    gap closest to the desired heading.
  - _compute_repulsion(): Potential-field repulsive vector from nearby obstacles.

Thread safety: Only called from the ROS control_loop timer (single-threaded).
All shared state access goes through locked stores.
"""

import math
import time

from geometry_msgs.msg import Twist

from .navigator import (
    VFH_AVOIDANCE_SPEED_FACTOR,
    EMERGENCY_BACKUP_FACTOR, VFH_BACKUP_SPEED_FACTOR, SLOWDOWN_MIN_FACTOR,
    STEERING_ANGULAR_GAIN, STEERING_ANGULAR_CLAMP,
    VFH_FUSED_ANGULAR_FACTOR, VFH_FUSED_ANGULAR_GAIN,
    VFH_ROTATE_ANGULAR_FACTOR, VFH_ROTATE_ANGULAR_GAIN,
    CLEAR_PATH_ANGULAR_FACTOR, CLEAR_PATH_ANGULAR_GAIN,
    VFH_COMMIT_TARGET_SECS,
    VFH_STUCK_TIMEOUT_SECS, LIDAR_STALE_TIMEOUT_SECS,
    OBS_LOG_INTERVAL_SECS, LOG_INTERVAL_SECS,
    VFH_NUM_BINS, AVOID_CLEAR_TICKS, LIDAR_MIN_POINT_DIST,
    VFH_CLEARANCE_MARGIN, VFH_BODY_MARGIN,
    FRONT_CORRIDOR_MARGIN, SIDE_CHECK_DEPTH,
    TARGET_NEAR_FACTOR,
)


class ObstacleAvoidanceMixin:
    """VFH obstacle avoidance mixin providing holonomic navigation with gap-finding.

    Implements a simplified Vector Field Histogram (VFH) algorithm adapted for
    a holonomic mecanum drive:

    1. **Polar histogram** -- The front 180 degrees of LiDAR points are binned
       into VFH_NUM_BINS angular sectors (5 deg each). Each bin records the
       closest obstacle distance.
    2. **Gap finding** -- A "gap" is a contiguous set of bins where all
       obstacles are farther than ``obstacle_distance + clearance_margin``
       and the gap is wide enough for the robot body. The gap whose center
       is closest to the desired heading is selected.
    3. **Direction commitment** -- Once an avoidance direction (left/right)
       is chosen, the robot commits to it for a minimum time to prevent
       oscillation between gaps.
    4. **Mecanum strafing** -- Unlike differential-drive VFH, the robot can
       strafe through the gap while independently rotating to keep the
       camera on target (fused mode).
    5. **Emergency backup** -- If no gap exists, the robot reverses slowly.
       A stuck timeout triggers a full stop after VFH_STUCK_TIMEOUT_SECS.

    Additionally provides ``check_obstacles()`` for full-body collision checks
    (front, rear, left, right corridors) and ``_compute_repulsion()`` for
    potential-field escape vectors used by the search spiral.
    """

    def direct_navigate(self, target_x, target_y, distance, target_visible=False):
        """Mecanum holonomic navigation with VFH obstacle avoidance.

        Uses Vector Field Histogram (VFH) gap-finding to steer around obstacles.
        When the target is visible, obstacles in the target direction are treated
        as the target itself (target/object) -- the robot drives toward it instead
        of avoiding it.

        Args:
            target_x: Target X position in robot (base_link) frame [m].
            target_y: Target Y position in robot (base_link) frame [m].
            distance: Euclidean distance to the target [m].
            target_visible: If True, the camera sees the target.
        """
        angle = math.atan2(target_y, target_x)
        cmd = Twist()
        now = time.time()

        min_front, left_free, right_free, *_ = self.check_obstacles()
        self._state.nav.set_obstacle(min_front < self.obstacle_distance, min_front)

        # When target is visible and close, the obstacle ahead IS the target.
        # Don't avoid it -- drive toward it.
        if target_visible and distance < self.obstacle_slowdown_distance * TARGET_NEAR_FACTOR:
            speed = self._speed_for_distance(distance)
            cmd.linear.x = speed * math.cos(angle)
            cmd.linear.y = speed * math.sin(angle)
            cmd.angular.z = max(-STEERING_ANGULAR_CLAMP, min(STEERING_ANGULAR_CLAMP, angle * STEERING_ANGULAR_GAIN))
            self._publish_cmd(cmd)
            return

        if min_front < self.obstacle_distance:
            # --- OBSTACLE: VFH avoidance ---
            if min_front < self.lidar_min_range:
                # Emergency: too close -- back up immediately + strafe to clear
                cmd.linear.x = -self.linear_speed * EMERGENCY_BACKUP_FACTOR
                if left_free:
                    cmd.linear.y = self.min_linear_speed
                elif right_free:
                    cmd.linear.y = -self.min_linear_speed
            else:
                self._stuck_start = 0.0
                gap_angle = self._find_best_gap(angle)

                if gap_angle is not None:
                    # Commit to avoidance direction (prevents oscillation)
                    commit_time = VFH_COMMIT_TARGET_SECS if target_visible else self._avoid_min_commit
                    gap_side = 1 if gap_angle > 0 else -1
                    if self._avoid_direction == 0:
                        self._avoid_direction = gap_side
                        self._avoid_start_time = now
                    elif now - self._avoid_start_time > commit_time:
                        self._avoid_direction = gap_side

                    if self._avoid_direction != gap_side:
                        committed_gap = self._find_committed_gap(angle)
                        if committed_gap is not None:
                            gap_angle = committed_gap

                    # Drive through the gap using mecanum strafing
                    speed = self.linear_speed * VFH_AVOIDANCE_SPEED_FACTOR
                    cmd.linear.x = speed * math.cos(gap_angle)
                    cmd.linear.y = speed * math.sin(gap_angle)

                    if target_visible:
                        # FUSED: keep camera on target while body strafes
                        target_ang = self.last_target_angle
                        cmd.angular.z = max(-self.angular_speed * VFH_FUSED_ANGULAR_FACTOR,
                                            min(self.angular_speed * VFH_FUSED_ANGULAR_FACTOR,
                                                target_ang * VFH_FUSED_ANGULAR_GAIN))
                    else:
                        # No target: rotate toward gap
                        cmd.angular.z = max(-self.angular_speed * VFH_ROTATE_ANGULAR_FACTOR,
                                            min(self.angular_speed * VFH_ROTATE_ANGULAR_FACTOR,
                                                gap_angle * VFH_ROTATE_ANGULAR_GAIN))
                else:
                    # No gap: back up (with stuck timeout)
                    cmd.linear.x = -self.linear_speed * VFH_BACKUP_SPEED_FACTOR
                    self._avoid_direction = 0
                    if self._stuck_start == 0.0:
                        self._stuck_start = now
                    elif now - self._stuck_start > VFH_STUCK_TIMEOUT_SECS:
                        self._logger.warning("VFH: stuck backing up >3s, stopping")
                        self.stop_robot()
                        return

            if now - self._last_obs_log > OBS_LOG_INTERVAL_SECS:
                fused = " [FUSED]" if target_visible else ""
                self._logger.warning(
                    f"OBSTACLE at {min_front:.2f}m! "
                    f"avoid={'L' if self._avoid_direction > 0 else 'R' if self._avoid_direction < 0 else '-'}"
                    f"{fused}")
                self._last_obs_log = now
        else:
            # --- CLEAR PATH ---
            if self._avoid_direction != 0:
                self._avoid_clear_count += 1
                if self._avoid_clear_count >= AVOID_CLEAR_TICKS:
                    self._avoid_direction = 0
                    self._avoid_clear_count = 0
            else:
                self._avoid_clear_count = 0

            self._stuck_start = 0.0
            speed = self._speed_for_distance(distance)
            cmd.linear.x = speed * math.cos(angle)
            cmd.linear.y = speed * math.sin(angle)
            cmd.angular.z = max(-self.angular_speed * CLEAR_PATH_ANGULAR_FACTOR,
                                min(self.angular_speed * CLEAR_PATH_ANGULAR_FACTOR, angle * CLEAR_PATH_ANGULAR_GAIN))

            # Slow down when LiDAR sees something in the slowdown zone
            if min_front < self.obstacle_slowdown_distance:
                denom = self.obstacle_slowdown_distance - self.obstacle_distance
                if denom > 0:
                    slow_factor = (min_front - self.obstacle_distance) / denom
                    slow = max(SLOWDOWN_MIN_FACTOR, min(1.0, slow_factor))
                    cmd.linear.x *= slow
                    cmd.linear.y *= slow

        # Side obstacle protection: don't strafe into walls
        if not left_free and cmd.linear.y > 0:
            cmd.linear.y = 0.0
        if not right_free and cmd.linear.y < 0:
            cmd.linear.y = 0.0

        self._publish_cmd(cmd)

    def _build_lidar_histogram(self):
        """Build a polar histogram of the front 180 deg from LiDAR points.

        Divides the front hemisphere into ``VFH_NUM_BINS`` equal-width angular
        bins and records the closest obstacle distance in each bin.

        Returns:
            Tuple (histogram, num_bins, bin_size) where *histogram* is a list
            of per-bin minimum distances, or ``None`` if no LiDAR data is
            available.
        """
        points, _, _ = self._state.sensors.get_laser()
        if len(points) == 0:
            return None

        num_bins = VFH_NUM_BINS
        bin_size = math.pi / num_bins
        histogram = [float('inf')] * num_bins

        for x, y in points:
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            dist = math.sqrt(x * x + y * y)
            if dist < LIDAR_MIN_POINT_DIST:
                continue
            ang = math.atan2(y, x)
            if abs(ang) > math.pi / 2:
                continue  # behind robot
            idx = int((ang + math.pi / 2) / bin_size)
            idx = max(0, min(num_bins - 1, idx))
            histogram[idx] = min(histogram[idx], dist)

        return histogram, num_bins, bin_size

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
        result = self._build_lidar_histogram()
        if result is None:
            return None
        histogram, num_bins, bin_size = result

        # Compute how many histogram bins the robot body subtends at the
        # clearance distance. A gap must span at least this many clear bins
        # for the robot to fit through without collision.
        clear_dist = self.obstacle_distance + VFH_CLEARANCE_MARGIN
        body_angle = 2.0 * math.atan2(self.robot_half_width + VFH_BODY_MARGIN, clear_dist)
        body_bins = max(1, int(math.ceil(body_angle / bin_size)))
        half_body = body_bins // 2

        # Evaluate each bin as a candidate travel direction. A direction is
        # passable if all bins within the robot body width are clear. Among
        # passable directions, pick the one closest to target_angle.
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

    def _find_committed_gap(self, target_angle):
        """Find the best gap restricted to the committed avoidance side.

        Same algorithm as ``_find_best_gap`` but only considers gaps on
        the side determined by ``_avoid_direction`` (left or right).

        Args:
            target_angle: Desired travel direction in robot frame [rad].

        Returns:
            Gap angle [rad] on the committed side, or None.
        """
        result = self._build_lidar_histogram()
        if result is None:
            return None
        histogram, num_bins, bin_size = result

        clear_dist = self.obstacle_distance + VFH_CLEARANCE_MARGIN
        body_angle = 2.0 * math.atan2(self.robot_half_width + VFH_BODY_MARGIN, clear_dist)
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

    def check_obstacles(self):
        """Check LiDAR for obstacles around the full robot body.

        Returns (min_front, left_free, right_free, min_rear, min_left, min_right).

        Uses the robot's actual dimensions to check all four sides:
        - Front: points within robot width, forward of center
        - Rear: points within robot width, behind center
        - Left: points within robot length, left of center
        - Right: points within robot length, right of center

        Fail-safe: returns (0.0, False, False, 0.0, 0.0, 0.0) if LiDAR data
        is stale or empty, which triggers emergency stop.
        """
        points, _, laser_time = self._state.sensors.get_laser()

        # Fail-safe: stale or missing LiDAR data -> assume obstacle everywhere
        if len(points) == 0 or (laser_time > 0 and time.time() - laser_time > LIDAR_STALE_TIMEOUT_SECS):
            if time.time() - self._last_obs_log > LOG_INTERVAL_SECS:
                self._logger.warning("LiDAR data stale or empty - fail-safe stop")
                self._last_obs_log = time.time()
            return 0.0, False, False, 0.0, 0.0, 0.0

        hw = self.robot_half_width
        hl = self._state.detection.get_robot_dims()['length'] / 2.0
        margin = FRONT_CORRIDOR_MARGIN
        slow_dist = self.obstacle_slowdown_distance
        min_front = float('inf')
        min_rear = float('inf')
        min_left = float('inf')
        min_right = float('inf')
        left_free = True
        right_free = True

        # Obstacle distances are measured from the robot EDGES, not center.
        # A point at x=0.24m with hl=0.15m is 0.09m from the front edge.
        # This ensures obstacle_distance (e.g., 0.22m) means 0.22m clearance
        # from the physical body, not 0.22m from the coordinate origin.
        for x, y in points:
            if not (math.isfinite(x) and math.isfinite(y)):
                continue

            # Front corridor: points ahead of the front edge, within body width
            if x > hl and x <= hl + slow_dist and abs(y) < hw + margin:
                min_front = min(min_front, x - hl)

            # Rear corridor: points behind the rear edge, within body width
            if x < -hl and x >= -(hl + slow_dist) and abs(y) < hw + margin:
                min_rear = min(min_rear, abs(x) - hl)

            # Left side: points beside robot (within robot length zone)
            if abs(x) < hl + margin and y > hw:
                if y < hw + SIDE_CHECK_DEPTH:
                    left_free = False
                min_left = min(min_left, y - hw)

            # Right side: points beside robot (within robot length zone)
            if abs(x) < hl + margin and y < -hw:
                if y > -(hw + SIDE_CHECK_DEPTH):
                    right_free = False
                min_right = min(min_right, abs(y) - hw)

        return min_front, left_free, right_free, min_rear, min_left, min_right

    def _compute_repulsion(self, influence_dist):
        """Potential field: compute repulsive vector from ALL nearby LiDAR points (360 deg).

        Each obstacle point within influence_dist contributes an inverse-square
        repulsive force pointing away from the obstacle. The result is the sum
        of all such forces, giving a vector that pushes the robot away from
        the nearest cluster of obstacles.

        Args:
            influence_dist: Maximum range (meters) at which obstacles exert force.

        Returns:
            Tuple (rx, ry) in robot frame -- the escape direction. Magnitude
            indicates urgency. Returns (0.0, 0.0) if no obstacles are nearby.
        """
        points, _, laser_time = self._state.sensors.get_laser()
        if len(points) == 0:
            return 0.0, 0.0

        rx, ry = 0.0, 0.0
        # Pre-compute the force offset so force drops to zero at influence_dist
        inv_inf2 = 1.0 / (influence_dist * influence_dist)

        for x, y in points:
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            d2 = x * x + y * y
            dist = math.sqrt(d2)
            if dist < LIDAR_MIN_POINT_DIST or dist > influence_dist:
                continue
            # Inverse-square repulsion minus the value at influence_dist,
            # so force is zero at the boundary and grows toward the obstacle.
            # The negative sign makes the vector point AWAY from the obstacle.
            force = (1.0 / d2) - inv_inf2
            rx -= (x / dist) * force
            ry -= (y / dist) * force

        return rx, ry
