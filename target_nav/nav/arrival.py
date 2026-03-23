"""Arrival detection mixin for Navigator.

Provides ArrivalMixin, which detects when the robot has reached the target.

Algorithm (simple and fast):
  1. Collect distance from ALL sources: robot-frame (camera tracker),
     map-frame (odom goal), and sensor distance (camera/LiDAR fusion).
  2. Take the SMALLEST distance (most optimistic).
  3. If smallest <= approach_distance for 0.3 seconds, declare ARRIVED.

Why short confirmation (0.3s):
  The robot moves at 0.10-0.20 m/s. With approach_distance=0.22m, the robot
  crosses the arrival zone in ~1.5s. A long confirmation (0.8s+) means the
  robot passes through and exits before arrival is detected.

Thread safety: Only called from the ROS control_loop timer (single-threaded).
"""

import math
import time

from geometry_msgs.msg import Twist


class ArrivalMixin:
    """Multi-signal arrival detection with temporal confirmation.

    Algorithm: collect distance from ALL sources (robot-frame, map-frame,
    sensor distance), take the SMALLEST, and declare ARRIVED if it stays
    within ``approach_distance`` for ``ARRIVAL_CONFIRM_SECS`` (0.3s).

    A hysteresis factor (1.5x) prevents the timer from resetting on minor
    distance fluctuations -- the robot must move significantly farther
    away before the arrival timer resets.
    """

    def _get_map_distance_to_target(self):
        """Compute distance from robot to target using odom-frame positions.

        This works even when LiDAR and camera are blind at close range,
        because it uses the odometry pose and the last-known target
        position in the odom frame.

        Returns:
            Distance in meters, or None if unavailable.
        """
        nav_target_map = self._state.nav.get_nav_target_map()
        if nav_target_map is None:
            return None
        robot_x, robot_y, _ = self._state.sensors.get_robot_pose()
        dx = nav_target_map[0] - robot_x
        dy = nav_target_map[1] - robot_y
        return math.sqrt(dx * dx + dy * dy)

    def _get_robot_frame_distance(self):
        """Get distance to target from robot-frame coordinates.

        Uses the target_robot_xy stored by nav_controller from
        camera+LiDAR matching. This is the most reliable distance
        because it comes directly from sensors, not odometry.

        Returns:
            Distance in meters, or None if no target.
        """
        target_bl = self._state.nav.get_target_robot_xy()
        if target_bl is None:
            return None
        return math.sqrt(target_bl[0]**2 + target_bl[1]**2)

    def _check_arrival(self, distance, angle, bbox_clipped=False):
        """Fast arrival detection using the best available distance signal.

        Uses a simple approach: check ALL distance sources, take the smallest
        (most optimistic), and declare arrival if it's within threshold.
        No complex scoring — just "is the robot close enough by ANY measure?"

        Confirmation time is short (0.3s) because the robot moves fast
        relative to the approach distance. At 0.15 m/s, the robot crosses
        the 22cm arrival zone in ~1.5s. A 0.8s confirmation would miss it.

        Distance sources (checked in order of reliability):
          1. Robot-frame distance (target_robot_xy from camera tracker)
          2. Map-frame distance (odom-frame goal vs robot pose)
          3. Sensor distance (camera estimated or LiDAR fused)

        Args:
            distance: Best available sensor distance to target [m].
            angle: Target bearing in base_link frame [rad].
            bbox_clipped: True if detection bbox touches frame edge.

        Returns:
            True if arrival is confirmed.
        """
        now = time.time()
        threshold = self.approach_distance

        # Collect all distance measurements
        best_dist = distance  # camera/fusion distance

        # Robot-frame distance (most direct — from camera+Kalman tracker)
        robot_dist = self._get_robot_frame_distance()
        if robot_dist is not None and robot_dist < best_dist:
            best_dist = robot_dist

        # Map-frame distance (works even when camera loses target)
        map_dist = self._get_map_distance_to_target()
        if map_dist is not None and map_dist < best_dist:
            best_dist = map_dist

        # Bbox clipped = target fills camera = very close
        # Trust camera distance less but still count it
        if bbox_clipped and distance < threshold * 3.0:
            best_dist = min(best_dist, distance)

        is_close = best_dist <= threshold

        if is_close:
            if self._arrival_first_seen == 0.0:
                self._arrival_first_seen = now
                self._logger.info(
                    f"Arrival zone: best={best_dist:.2f}m "
                    f"(robot={robot_dist:.2f if robot_dist else 'N/A'}m "
                    f"map={map_dist:.2f if map_dist else 'N/A'}m "
                    f"sensor={distance:.2f}m) thresh={threshold:.2f}m")
            elif now - self._arrival_first_seen >= self._arrival_confirm_time:
                self._logger.info(
                    f"ARRIVED: best={best_dist:.2f}m thresh={threshold:.2f}m "
                    f"confirmed {self._arrival_confirm_time:.1f}s")
                return True
        else:
            # Reset only if clearly outside (prevents oscillation at boundary)
            if best_dist > threshold * 1.5:
                self._arrival_first_seen = 0.0

        return False

    def _arrive(self):
        """Execute arrival sequence: stop robot, set ARRIVED state, clear timers."""
        cmd = Twist()
        self._publish_cmd(cmd)
        self._state.nav.set_nav_state("ARRIVED")
        self._state.nav.set_obstacle(False, float('inf'))
        self._arrival_first_seen = 0.0
