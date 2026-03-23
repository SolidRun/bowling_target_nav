"""Search behavior mixins for Navigator: 360-degree rotation scan and spiral search.

Provides SearchMixin, which implements two tiers of target search when the
detection pipeline loses sight of the target:

  - **Tier 1 (360-degree scan)**: In-place rotation scan with potential-field
    obstacle repulsion. Rotates in the direction of the last known target angle.
    Completes when cumulative rotation reaches 360 degrees.

  - **Tier 2 (Archimedean spiral)**: Expanding spiral search centered on the
    robot's position when Tier 1 finishes. The radius grows linearly per
    revolution. A sinusoidal angular oscillation sweeps the camera for wider
    detection coverage. VFH gap-finding provides obstacle avoidance during
    the spiral.

Thread safety: Only called from the ROS control_loop timer (single-threaded).
"""

import math
import time

from geometry_msgs.msg import Twist

from .navigator import (
    VFH_CLEARANCE_MARGIN, VFH_BACKUP_SPEED_FACTOR,
    SPIRAL_AVOIDANCE_SPEED_FACTOR,
    SPIRAL_DT_CLAMP, SPIRAL_MIN_RADIUS, SPIRAL_MIN_ANGULAR_RATE,
    SPIRAL_WAYPOINT_THRESHOLD, SPIRAL_SWEEP_FREQ,
    REPULSION_MIN_MAG,
)


class SearchMixin:
    """Tier 1 (360-degree scan) and Tier 2 (spiral) search methods for Navigator.

    Two-tier search strategy:

    **Tier 1 -- 360-degree in-place scan:**
      The robot rotates in place toward the last known target direction.
      Cumulative rotation is tracked via odometry yaw deltas. During the
      rotation, a potential-field repulsive vector from nearby LiDAR points
      nudges the robot away from walls. Completes when cumulative rotation
      reaches 360 degrees. If the target is re-detected during the scan,
      navigation resumes immediately (handled by nav_controller).

    **Tier 2 -- Archimedean spiral search:**
      If Tier 1 fails, the robot follows an expanding spiral path centered
      on its current position. The spiral follows the Archimedean equation:

        r(theta) = r_initial + growth_rate * theta / (2 * pi)

      This means the radius increases by ``growth_rate`` meters per full
      revolution. The angular rate is derived from linear speed:

        omega = linear_speed / r(theta)

      A sinusoidal angular oscillation (``SPIRAL_SWEEP_FREQ`` Hz) is
      superimposed on the heading so the camera sweeps left-right during
      the spiral, increasing detection coverage. VFH obstacle avoidance
      runs throughout. The spiral terminates on timeout or when the
      maximum radius is reached.
    """

    def start_search_scan(self):
        """Initialize a 360-degree search rotation.

        Records the current yaw as the starting reference and resets
        the cumulative rotation counter.
        """
        _, _, yaw = self._state.sensors.get_robot_pose()
        self._search_start_yaw = yaw
        self._search_last_yaw = yaw
        self._search_total_rotated = 0.0
        self._search_initialized = True
        self._logger.info(
            f"360\u00b0 search scan started at yaw={math.degrees(yaw):.1f}\u00b0")

    def search_rotate(self):
        """Execute one tick of 360-degree search scan.

        Rotates in-place toward the last known target direction while applying
        potential-field repulsion from nearby obstacles. Tracks cumulative
        rotation via odometry yaw deltas.

        Returns:
            True when the full 360-degree rotation is complete, False otherwise.
        """
        if not self._search_initialized:
            self.start_search_scan()

        # Track rotation via odometry
        _, _, current_yaw = self._state.sensors.get_robot_pose()
        delta = math.atan2(
            math.sin(current_yaw - self._search_last_yaw),
            math.cos(current_yaw - self._search_last_yaw))
        self._search_total_rotated += abs(delta)
        self._search_last_yaw = current_yaw

        # Check if we've completed 360deg
        if self._search_total_rotated >= 2 * math.pi:
            self.stop_robot()
            self._search_initialized = False
            self._logger.info("360\u00b0 search scan complete - target not found")
            return True  # Search complete, target not found

        # Fixed rotation direction (toward last known target angle)
        direction = 1.0 if self.last_target_angle >= 0 else -1.0

        cmd = Twist()
        cmd.angular.z = self.search_angular_speed * direction

        # Potential field: repulsive force from ALL nearby obstacles (360deg)
        rep_x, rep_y = self._compute_repulsion(self.obstacle_distance + VFH_CLEARANCE_MARGIN)
        rep_mag = math.sqrt(rep_x * rep_x + rep_y * rep_y)
        if rep_mag > REPULSION_MIN_MAG:
            cmd.linear.x = self.min_linear_speed * (rep_x / rep_mag)
            cmd.linear.y = self.min_linear_speed * (rep_y / rep_mag)

        self._publish_cmd(cmd)
        return False  # Still searching

    # ------------------------------------------------------------------
    # Spiral search (Tier 2: expanding spiral after 360-degree scan fails)
    # ------------------------------------------------------------------

    def start_spiral_search(self):
        """Initialize Tier 2 expanding Archimedean spiral search from current position.

        Records the robot's current position as the spiral center and sets
        the initial radius. Called after a 360-degree scan fails to find
        the target.
        """
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
            self._logger.warning(f"Spiral search timeout after {elapsed:.1f}s")
            self.stop_robot()
            return True

        if self._spiral_radius > self.spiral_max_radius:
            self._logger.warning(
                f"Spiral search max radius {self.spiral_max_radius:.1f}m reached")
            self.stop_robot()
            return True

        rx, ry, rt = self._state.sensors.get_robot_pose()

        # Advance spiral: angular rate = linear_speed / radius
        dt = min(now - self._spiral_last_time, SPIRAL_DT_CLAMP)
        self._spiral_last_time = now
        if self._spiral_radius > SPIRAL_MIN_RADIUS:
            angular_rate = self.spiral_linear_speed / self._spiral_radius
        else:
            angular_rate = SPIRAL_MIN_ANGULAR_RATE
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

        # Convert map-frame waypoint to robot-frame displacement
        # by rotating the difference vector by -robot_theta
        dx, dy = tx - rx, ty - ry
        cos_r, sin_r = math.cos(-rt), math.sin(-rt)
        local_x = dx * cos_r - dy * sin_r
        local_y = dx * sin_r + dy * cos_r

        # Obstacle avoidance
        min_front, left_free, right_free, min_rear, min_left, min_right = self.check_obstacles()
        cmd = Twist()

        if min_front < self.obstacle_distance:
            move_angle = math.atan2(local_y, local_x)
            gap = self._find_best_gap(move_angle)
            if gap is not None:
                speed = self.spiral_linear_speed * SPIRAL_AVOIDANCE_SPEED_FACTOR
                cmd.linear.x = speed * math.cos(gap)
                cmd.linear.y = speed * math.sin(gap)
            else:
                cmd.linear.x = -self.spiral_linear_speed * VFH_BACKUP_SPEED_FACTOR
        else:
            dist = math.sqrt(local_x ** 2 + local_y ** 2)
            if dist > SPIRAL_WAYPOINT_THRESHOLD:
                move_angle = math.atan2(local_y, local_x)
                cmd.linear.x = self.spiral_linear_speed * math.cos(move_angle)
                cmd.linear.y = self.spiral_linear_speed * math.sin(move_angle)
            else:
                cmd.linear.x = self.spiral_linear_speed * SPIRAL_AVOIDANCE_SPEED_FACTOR

        # Full body obstacle protection (all 4 directions)
        # Prevent moving toward any obstacle closer than obstacle_distance
        obs_d = self.obstacle_distance
        if min_rear < obs_d and cmd.linear.x < 0:
            cmd.linear.x = 0.0  # don't reverse into obstacle
        if not left_free and cmd.linear.y > 0:
            cmd.linear.y = 0.0  # don't strafe left into obstacle
        if not right_free and cmd.linear.y < 0:
            cmd.linear.y = 0.0  # don't strafe right into obstacle

        # Camera sweep: sinusoidal yaw oscillation so the camera scans
        # left and right during the spiral, increasing detection coverage
        sweep_phase = elapsed * SPIRAL_SWEEP_FREQ
        cmd.angular.z = self.spiral_angular_speed * math.sin(sweep_phase)

        # Enforce minimum speed (motor dead zone)
        total_speed = math.sqrt(cmd.linear.x ** 2 + cmd.linear.y ** 2)
        if total_speed > 0 and total_speed < self.min_linear_speed:
            scale = self.min_linear_speed / total_speed
            cmd.linear.x *= scale
            cmd.linear.y *= scale

        self._publish_cmd(cmd)
        return False  # Still searching
