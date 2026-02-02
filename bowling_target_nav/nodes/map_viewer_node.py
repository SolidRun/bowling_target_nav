#!/usr/bin/env python3
"""
Professional Map Viewer Node
============================

Real-time SLAM visualization with:
- Occupancy grid map
- Robot position and orientation
- LiDAR scan overlay
- Status panel

Usage:
    ros2 run bowling_target_nav map_viewer_node
"""

import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException


class MapViewerNode(Node):
    """Professional SLAM map visualization."""

    # Colors (BGR)
    COLOR_FREE = (50, 50, 50)        # Dark gray - free space
    COLOR_OCCUPIED = (0, 200, 255)   # Orange - obstacles
    COLOR_UNKNOWN = (30, 30, 30)     # Very dark - unknown
    COLOR_ROBOT = (0, 255, 0)        # Green - robot
    COLOR_LASER = (0, 0, 255)        # Red - laser points
    COLOR_GRID = (60, 60, 60)        # Grid lines
    COLOR_TEXT = (255, 255, 255)     # White text
    COLOR_PANEL = (40, 40, 40)       # Panel background

    def __init__(self):
        super().__init__('map_viewer_node')

        # Parameters
        self.declare_parameter('window_width', 1024)
        self.declare_parameter('window_height', 768)
        self.declare_parameter('robot_size', 15)

        self.window_width = self.get_parameter('window_width').value
        self.window_height = self.get_parameter('window_height').value
        self.robot_size = self.get_parameter('robot_size').value

        # State
        self.map_img = None
        self.map_info = None
        self.laser_points = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.scan_count = 0
        self.map_count = 0

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, sensor_qos)

        # Display timer (30 FPS)
        self.timer = self.create_timer(0.033, self._update_display)

        # Create window (not fullscreen for Wayland compatibility)
        cv2.namedWindow('SLAM Map', cv2.WINDOW_AUTOSIZE)

        self.get_logger().info('Professional Map Viewer started')

    def _map_callback(self, msg: OccupancyGrid):
        """Process map data."""
        self.map_info = msg.info
        self.map_count += 1

        width = msg.info.width
        height = msg.info.height

        # Convert to numpy array
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Create color image
        img = np.zeros((height, width, 3), dtype=np.uint8)

        # Color mapping
        img[data == -1] = self.COLOR_UNKNOWN   # Unknown
        img[data == 0] = self.COLOR_FREE       # Free
        img[data > 0] = self.COLOR_OCCUPIED    # Occupied

        # Flip (ROS origin is bottom-left)
        self.map_img = cv2.flip(img, 0)

    def _scan_callback(self, msg: LaserScan):
        """Process laser scan."""
        self.scan_count += 1

        # Convert scan to points
        points = []
        angle = msg.angle_min

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))
            angle += msg.angle_increment

        self.laser_points = points

        # Get robot pose from TF
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())

            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y

            # Quaternion to yaw
            q = transform.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_theta = math.atan2(siny_cosp, cosy_cosp)

        except TransformException:
            pass

    def _world_to_pixel(self, wx, wy):
        """Convert world coordinates to pixel coordinates."""
        if self.map_info is None:
            return None, None

        res = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        height = self.map_info.height

        px = int((wx - origin_x) / res)
        py = int(height - (wy - origin_y) / res)

        return px, py

    def _draw_robot(self, img, scale):
        """Draw robot position and orientation."""
        if self.map_info is None:
            return

        px, py = self._world_to_pixel(self.robot_x, self.robot_y)
        if px is None:
            return

        # Scale to display
        px = int(px * scale)
        py = int(py * scale)

        size = self.robot_size

        # Robot body (circle)
        cv2.circle(img, (px, py), size, self.COLOR_ROBOT, -1)
        cv2.circle(img, (px, py), size, (255, 255, 255), 2)

        # Direction arrow
        arrow_len = size * 2
        ax = int(px + arrow_len * math.cos(-self.robot_theta))
        ay = int(py + arrow_len * math.sin(-self.robot_theta))
        cv2.arrowedLine(img, (px, py), (ax, ay), (255, 255, 255), 3, tipLength=0.4)

    def _draw_laser(self, img, scale):
        """Draw laser scan points."""
        if self.map_info is None or not self.laser_points:
            return

        cos_t = math.cos(self.robot_theta)
        sin_t = math.sin(self.robot_theta)

        for lx, ly in self.laser_points:
            # Transform to world frame
            wx = self.robot_x + lx * cos_t - ly * sin_t
            wy = self.robot_y + lx * sin_t + ly * cos_t

            px, py = self._world_to_pixel(wx, wy)
            if px is None:
                continue

            px = int(px * scale)
            py = int(py * scale)

            if 0 <= px < img.shape[1] and 0 <= py < img.shape[0]:
                cv2.circle(img, (px, py), 2, self.COLOR_LASER, -1)

    def _draw_grid(self, img, scale):
        """Draw meter grid overlay."""
        if self.map_info is None:
            return

        res = self.map_info.resolution
        pixels_per_meter = int(1.0 / res * scale)

        if pixels_per_meter < 20:
            return

        h, w = img.shape[:2]

        # Vertical lines
        for x in range(0, w, pixels_per_meter):
            cv2.line(img, (x, 0), (x, h), self.COLOR_GRID, 1)

        # Horizontal lines
        for y in range(0, h, pixels_per_meter):
            cv2.line(img, (0, y), (w, y), self.COLOR_GRID, 1)

    def _draw_status_panel(self, img):
        """Draw status information panel."""
        panel_height = 80
        panel_width = 350

        # Panel background
        cv2.rectangle(img, (10, 10), (10 + panel_width, 10 + panel_height),
                      self.COLOR_PANEL, -1)
        cv2.rectangle(img, (10, 10), (10 + panel_width, 10 + panel_height),
                      (100, 100, 100), 2)

        # Title
        cv2.putText(img, 'SLAM Mapping', (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Status info
        if self.map_info:
            res = self.map_info.resolution
            w = self.map_info.width
            h = self.map_info.height
            info1 = f'Map: {w}x{h} @ {res:.2f}m/px'
        else:
            info1 = 'Map: Waiting...'

        info2 = f'Robot: ({self.robot_x:.2f}, {self.robot_y:.2f}) {math.degrees(self.robot_theta):.0f}deg'
        info3 = f'Scans: {self.scan_count} | Maps: {self.map_count}'

        cv2.putText(img, info1, (20, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLOR_TEXT, 1)
        cv2.putText(img, info2, (20, 72),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLOR_TEXT, 1)
        cv2.putText(img, info3, (200, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        # Legend
        legend_x = img.shape[1] - 150
        cv2.rectangle(img, (legend_x, 10), (img.shape[1] - 10, 80),
                      self.COLOR_PANEL, -1)
        cv2.putText(img, 'Legend:', (legend_x + 10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLOR_TEXT, 1)

        cv2.circle(img, (legend_x + 20, 45), 6, self.COLOR_OCCUPIED, -1)
        cv2.putText(img, 'Obstacle', (legend_x + 35, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.COLOR_TEXT, 1)

        cv2.circle(img, (legend_x + 20, 65), 6, self.COLOR_ROBOT, -1)
        cv2.putText(img, 'Robot', (legend_x + 35, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.COLOR_TEXT, 1)

    def _update_display(self):
        """Update display."""
        # Create display image
        display = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8)
        display[:] = self.COLOR_UNKNOWN

        if self.map_img is not None:
            # Calculate scale to fit window (with padding for panel)
            map_h, map_w = self.map_img.shape[:2]

            available_h = self.window_height - 100
            available_w = self.window_width - 20

            scale = min(available_w / map_w, available_h / map_h)

            # Resize map
            new_w = int(map_w * scale)
            new_h = int(map_h * scale)

            resized_map = cv2.resize(self.map_img, (new_w, new_h),
                                     interpolation=cv2.INTER_NEAREST)

            # Center map
            x_offset = (self.window_width - new_w) // 2
            y_offset = 90 + (available_h - new_h) // 2

            # Draw grid
            self._draw_grid(resized_map, scale)

            # Draw laser points
            self._draw_laser(resized_map, scale)

            # Draw robot
            self._draw_robot(resized_map, scale)

            # Place map in display
            display[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized_map

        else:
            # Waiting message
            cv2.putText(display, 'Waiting for map data...',
                        (self.window_width//2 - 200, self.window_height//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (100, 100, 100), 2)

        # Draw status panel
        self._draw_status_panel(display)

        # Instructions
        cv2.putText(display, 'Press Q to quit',
                    (self.window_width - 150, self.window_height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)

        cv2.imshow('SLAM Map', display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:  # q or ESC
            self.get_logger().info('Quit requested')
            raise SystemExit

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapViewerNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
