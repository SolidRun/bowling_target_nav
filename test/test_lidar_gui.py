#!/usr/bin/env python3
"""Minimal LiDAR radar display — test if scans update in real-time.

Shows raw LiDAR points as a radar view centered on the robot.
No Cartographer, no SLAM, no overlays — just points.

Usage (on V2N):
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=0
    python3 test/test_lidar_gui.py
"""

import math
import threading
import time

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import numpy as np
import cairo


class LidarTestWindow(Gtk.Window):

    def __init__(self):
        super().__init__(title="LiDAR Test")
        self.set_default_size(600, 600)
        self.connect("destroy", Gtk.main_quit)
        self.connect("key-press-event", self._on_key)

        self._points = np.empty((0, 2), dtype=np.float32)
        self._scan_count = 0
        self._scan_hz = 0.0
        self._last_scan_time = 0.0
        self._lock = threading.Lock()

        self._da = Gtk.DrawingArea()
        self._da.connect("draw", self._on_draw)
        self.add(self._da)

        # Redraw at 15fps
        GLib.timeout_add(66, self._tick)

    def update_scan(self, points):
        """Called from ROS thread with new scan points in base_link frame."""
        now = time.time()
        with self._lock:
            self._points = points
            self._scan_count += 1
            if self._last_scan_time > 0:
                dt = now - self._last_scan_time
                if 0 < dt < 2:
                    self._scan_hz = 0.3 * (1/dt) + 0.7 * self._scan_hz
            self._last_scan_time = now

    def _tick(self):
        self._da.queue_draw()
        return True

    def _on_key(self, widget, event):
        from gi.repository import Gdk
        key = Gdk.keyval_name(event.keyval)
        if key and key.lower() in ('q', 'escape'):
            Gtk.main_quit()

    def _on_draw(self, da, cr):
        w = da.get_allocated_width()
        h = da.get_allocated_height()

        # Black background
        cr.set_source_rgb(0.05, 0.05, 0.08)
        cr.rectangle(0, 0, w, h)
        cr.fill()

        cx = w / 2
        cy = h / 2
        radius = 5.0  # 5m view
        ppm = min(w, h) / (2 * radius)

        # Grid circles (1m, 2m, 3m, 4m, 5m)
        cr.set_source_rgba(0.2, 0.25, 0.3, 0.5)
        cr.set_line_width(0.5)
        for r in range(1, 6):
            cr.arc(cx, cy, r * ppm, 0, 2 * math.pi)
            cr.stroke()

        # Grid labels
        cr.set_source_rgba(0.4, 0.5, 0.6, 0.7)
        cr.set_font_size(10)
        for r in range(1, 6):
            cr.move_to(cx + 3, cy - r * ppm + 12)
            cr.show_text(f"{r}m")

        # Cross at center
        cr.set_source_rgba(0.3, 0.4, 0.3, 0.5)
        cr.set_line_width(0.5)
        cr.move_to(cx, 0)
        cr.line_to(cx, h)
        cr.stroke()
        cr.move_to(0, cy)
        cr.line_to(w, cy)
        cr.stroke()

        with self._lock:
            pts = self._points.copy()
            count = self._scan_count
            hz = self._scan_hz

        # Draw points
        if len(pts) > 0:
            cr.set_source_rgb(0.0, 1.0, 0.6)
            for i in range(len(pts)):
                px, py = pts[i]
                sx = cx - py * ppm   # Y = left/right (negative = right on screen)
                sy = cy - px * ppm   # X = forward (up on screen)
                cr.rectangle(sx - 1, sy - 1, 3, 3)
            cr.fill()

        # Robot dot
        cr.set_source_rgb(0.2, 1.0, 0.4)
        cr.arc(cx, cy, 5, 0, 2 * math.pi)
        cr.fill()

        # Forward arrow
        cr.set_source_rgb(1.0, 1.0, 0.3)
        cr.set_line_width(2)
        cr.move_to(cx, cy)
        cr.line_to(cx, cy - 25)
        cr.stroke()
        cr.move_to(cx - 5, cy - 20)
        cr.line_to(cx, cy - 25)
        cr.line_to(cx + 5, cy - 20)
        cr.stroke()

        # Stats
        cr.set_source_rgb(0.0, 0.8, 1.0)
        cr.set_font_size(14)
        cr.move_to(10, 20)
        cr.show_text(f"LiDAR Test  |  {len(pts)} points  |  {hz:.1f} Hz  |  scan #{count}")

        cr.set_source_rgb(0.5, 0.6, 0.7)
        cr.set_font_size(11)
        cr.move_to(10, h - 10)
        cr.show_text("Press Q or ESC to quit")


def main():
    rclpy.init()
    node = Node("lidar_test")
    win = LidarTestWindow()

    def scan_cb(msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)
        angles = np.linspace(msg.angle_min, msg.angle_min + (n-1) * msg.angle_increment, n)
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        r = ranges[valid]
        a = angles[valid]
        if len(r) > 0:
            x = r * np.cos(a)
            y = r * np.sin(a)

            # Apply LiDAR yaw rotation (180 degrees)
            lidar_yaw = math.pi  # 180 degrees
            cos_ly = math.cos(lidar_yaw)
            sin_ly = math.sin(lidar_yaw)
            xb = x * cos_ly - y * sin_ly
            yb = x * sin_ly + y * cos_ly
            pts = np.column_stack((xb, yb))
        else:
            pts = np.empty((0, 2), dtype=np.float32)
        win.update_scan(pts)

    node.create_subscription(LaserScan, '/scan', scan_cb, 10)

    # Spin ROS in background thread
    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    t = threading.Thread(target=ros_spin, daemon=True)
    t.start()

    win.show_all()
    Gtk.main()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
