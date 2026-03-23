"""Blind approach (dead-reckoning) mixin for Navigator.

Provides BlindApproachMixin, which implements dead-reckoning navigation
toward the last known target position when the camera loses sight of the
target at close range. This handles the common case where the target
passes below the camera's field of view as the robot gets close.

The approach works by converting the last base_link-frame target position
to odom coordinates using odometry, then driving toward those odom coordinates
using holonomic motion with heading correction.

Exit conditions: arrival (map distance), timeout, LiDAR obstacle, heading
divergence, or stale odometry.

Thread safety: Only called from the ROS control_loop timer (single-threaded).
"""

import math
import time

from geometry_msgs.msg import Twist

from .navigator import (
    BLIND_ANGULAR_CLAMP, BLIND_ANGULAR_GAIN,
    BLIND_OBSTACLE_SPEED_FACTOR, BLIND_MAX_HEADING_DEG,
    BLIND_SPEED_RAMP_DIST, BLIND_LIDAR_ARRIVAL_FACTOR,
)


class BlindApproachMixin:
    """Dead-reckoning blind approach mixin for close-range target acquisition.

    When and why blind approach is used:
      The target is typically lost at close range because (a) it passes below
      the camera's field of view, or (b) it falls within the LiDAR's minimum
      range (~0.15 m for RPLidar A1). At this point the robot has a reliable
      odom-frame goal from previous detections, so it can dead-reckon the
      remaining distance using odometry alone.

    Entry conditions (checked by ``navigate_to_target``):
      - Distance to target < ``blind_approach_entry_distance``
        (defaults to ``obstacle_slowdown_distance``).
      - Robot pose is not stale (not default 0,0,0).
      - An odom-frame map goal exists (set by prior camera detections).

    Exit conditions (checked each tick by ``blind_approach_step``):
      - **Arrived**: odom-frame distance <= ``blind_approach_arrival_margin``.
      - **Timeout**: elapsed time > ``blind_approach_timeout`` (default 10s).
      - **LiDAR obstacle**: front obstacle < ``blind_approach_lidar_stop``.
        If close to target, declares ARRIVED; otherwise falls back to search.
      - **Heading divergence**: heading error > 45 degrees (target likely
        not ahead anymore).
      - **Stale pose**: odometry stuck at (0,0,0).
    """

    def enter_blind_approach(self):
        """Enter blind approach mode: dead-reckon to stored odom-frame goal.

        Uses the odom-frame target already computed by navigate_to_target()
        instead of re-computing from base_link, avoiding coordinate
        conversion drift.

        Returns:
            True if blind approach was successfully entered, False if the
            robot pose is stale or no map goal is available.
        """
        robot_x, robot_y, robot_theta = self._state.sensors.get_robot_pose()

        # Reject if pose looks like default (0,0,0) - likely stale/unavailable
        if robot_x == 0.0 and robot_y == 0.0 and robot_theta == 0.0:
            self._logger.warning("BLIND_APPROACH: rejected - robot pose is default (0,0,0)")
            return False

        # Use the already-stored map goal (set by navigate_to_target)
        map_goal = self._state.nav.get_nav_target_map()
        if map_goal is None:
            # Fallback: compute from base_link nav target
            _, nav_target = self._state.nav.get_nav_state()
            if nav_target is None:
                return False
            target_x_bl, target_y_bl, distance = nav_target
            cos_t = math.cos(robot_theta)
            sin_t = math.sin(robot_theta)
            target_x_map = robot_x + target_x_bl * cos_t - target_y_bl * sin_t
            target_y_map = robot_y + target_x_bl * sin_t + target_y_bl * cos_t
        else:
            target_x_map, target_y_map = map_goal
            dx = target_x_map - robot_x
            dy = target_y_map - robot_y
            distance = math.sqrt(dx * dx + dy * dy)

        self.blind_approach_active = True
        self.blind_approach_start_time = time.time()
        self.blind_approach_start_pose = (robot_x, robot_y, robot_theta)
        self.blind_approach_target_map = (target_x_map, target_y_map)
        self.blind_approach_target_distance = distance

        _, nav_target = self._state.nav.get_nav_state()
        self._state.nav.set_nav_state("BLIND_APPROACH", nav_target)
        self._logger.info(
            f"BLIND_APPROACH: target_map=({target_x_map:.2f}, {target_y_map:.2f}), "
            f"distance={distance:.2f}m")
        return True

    def blind_approach_step(self):
        """Execute one tick of blind approach dead-reckoning.

        Computes the remaining distance and heading error to the stored
        odom-frame target, then drives holonomically toward it.  Exits on
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
            self._logger.warning("BLIND_APPROACH: pose is (0,0,0) - aborting")
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

        # Exit: arrived -- use odom-frame distance (same as _check_arrival)
        map_dist = self._get_map_distance_to_target()
        best_remaining = map_dist if map_dist is not None else remaining
        if best_remaining <= self.blind_approach_arrival_margin:
            self._logger.info(
                f"BLIND_APPROACH: Arrived! map_dist="
                f"{'%.3f' % map_dist if map_dist is not None else 'N/A'}m, "
                f"odom_remaining={remaining:.3f}m")
            self.stop_robot()
            self.blind_approach_active = False
            self._state.nav.set_nav_state("ARRIVED")
            return

        # Exit: timeout
        if elapsed > self.blind_approach_timeout:
            self._logger.warning(f"BLIND_APPROACH: Timeout after {elapsed:.1f}s")
            self.stop_robot()
            self.blind_approach_active = False
            self._state.nav.start_search()
            return

        # Exit: LiDAR obstacle -- only declare ARRIVED if close to target
        min_front, left_free, right_free, *_ = self.check_obstacles()
        lidar_stop = max(self.blind_approach_lidar_stop, self.lidar_min_range)
        if min_front < lidar_stop:
            if best_remaining < self.blind_approach_arrival_margin * BLIND_LIDAR_ARRIVAL_FACTOR:
                self._logger.info(
                    f"BLIND_APPROACH: LiDAR stop at {min_front:.3f}m, "
                    f"remaining={best_remaining:.3f}m -> ARRIVED")
                self.stop_robot()
                self.blind_approach_active = False
                self._state.nav.set_nav_state("ARRIVED")
                return
            else:
                self._logger.warning(
                    f"BLIND_APPROACH: LiDAR obstacle at {min_front:.3f}m but "
                    f"remaining={best_remaining:.2f}m -> search")
                self.stop_robot()
                self.blind_approach_active = False
                self._state.nav.start_search()
                return

        # Exit: heading diverged
        if abs(heading_error) > math.radians(BLIND_MAX_HEADING_DEG):
            self._logger.warning(f"BLIND_APPROACH: Heading diverged {math.degrees(heading_error):.1f}deg")
            self.stop_robot()
            self.blind_approach_active = False
            self._state.nav.start_search()
            return

        # Drive with mecanum holonomic motion
        cmd = Twist()
        speed_ramp = min(1.0, remaining / BLIND_SPEED_RAMP_DIST)
        speed = max(self.min_linear_speed, self.blind_approach_speed * speed_ramp)
        cmd.linear.x = speed * math.cos(heading_error)
        cmd.linear.y = speed * math.sin(heading_error)
        cmd.angular.z = max(-BLIND_ANGULAR_CLAMP, min(BLIND_ANGULAR_CLAMP, BLIND_ANGULAR_GAIN * heading_error))

        # Side obstacle protection
        if not left_free and cmd.linear.y > 0:
            cmd.linear.y = 0.0
        if not right_free and cmd.linear.y < 0:
            cmd.linear.y = 0.0
        # Slow down when front obstacle is close
        if min_front < self.obstacle_distance:
            cmd.linear.x *= BLIND_OBSTACLE_SPEED_FACTOR

        self._publish_cmd(cmd)
