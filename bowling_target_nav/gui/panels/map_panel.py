"""SLAM map panel rendering."""

import math

import cv2
import numpy as np

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, GdkPixbuf

# Colors (BGR for OpenCV)
COLOR_FREE = (50, 50, 50)
COLOR_OCCUPIED = (0, 200, 255)
COLOR_UNKNOWN = (30, 30, 30)
COLOR_ROBOT = (0, 255, 0)
COLOR_LASER = (0, 0, 255)
COLOR_GRID = (60, 60, 60)


def draw_map_panel(cr, x, y, w, h, state):
    """Draw SLAM map panel with robot, laser, and navigation target overlay."""
    # Panel background
    cr.set_source_rgb(0.086, 0.106, 0.133)
    cr.rectangle(x, y, w, h)
    cr.fill()

    # Border
    cr.set_source_rgb(0.188, 0.212, 0.239)
    cr.set_line_width(1.5)
    cr.rectangle(x, y, w, h)
    cr.stroke()

    # Header
    cr.set_source_rgb(0.345, 0.651, 1.0)
    cr.set_font_size(15)
    cr.move_to(x + 10, y + 20)
    cr.show_text("SLAM Map")

    local_map, info, mc = state.sensors.get_map()
    rx, ry, rt = state.sensors.get_robot_pose()
    lpoints, sc = state.sensors.get_laser()

    content_y = y + 35
    content_h = h - 55

    if local_map is not None and info is not None:
        mh, mw = local_map.shape[:2]
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        scale = min((w - 20) / mw, content_h / mh)
        disp_w = int(mw * scale)
        disp_h = int(mh * scale)
        disp_x = x + (w - disp_w) // 2
        disp_y = content_y + (content_h - disp_h) // 2

        disp_map = cv2.resize(local_map, (disp_w, disp_h), interpolation=cv2.INTER_NEAREST)

        # Grid lines
        ppm = int(1.0 / res * scale)
        if ppm > 15:
            for gx in range(0, disp_w, ppm):
                cv2.line(disp_map, (gx, 0), (gx, disp_h), COLOR_GRID, 1)
            for gy in range(0, disp_h, ppm):
                cv2.line(disp_map, (0, gy), (disp_w, gy), COLOR_GRID, 1)

        # Laser points
        cos_t, sin_t = math.cos(rt), math.sin(rt)
        for lx, ly in lpoints:
            wx = rx + lx * cos_t - ly * sin_t
            wy = ry + lx * sin_t + ly * cos_t
            px = int((wx - ox) / res * scale)
            py = int(disp_h - (wy - oy) / res * scale)
            if 0 <= px < disp_w and 0 <= py < disp_h:
                cv2.circle(disp_map, (px, py), 2, COLOR_LASER, -1)

        # Navigation target and line
        nav_target_map = state.nav.get_nav_target_map()
        if nav_target_map is not None:
            tx_map, ty_map = nav_target_map
            tpx = int((tx_map - ox) / res * scale)
            tpy = int(disp_h - (ty_map - oy) / res * scale)
            rpx_t = int((rx - ox) / res * scale)
            rpy_t = int(disp_h - (ry - oy) / res * scale)
            if 0 <= tpx < disp_w and 0 <= tpy < disp_h:
                cv2.line(disp_map, (rpx_t, rpy_t), (tpx, tpy), (255, 100, 255), 1)
                pts = np.array([
                    [tpx, tpy - 8], [tpx + 8, tpy],
                    [tpx, tpy + 8], [tpx - 8, tpy]
                ], dtype=np.int32)
                cv2.fillPoly(disp_map, [pts], (255, 0, 255))
                cv2.polylines(disp_map, [pts], True, (255, 255, 255), 1)

        # Robot marker
        rpx = int((rx - ox) / res * scale)
        rpy = int(disp_h - (ry - oy) / res * scale)
        if 0 <= rpx < disp_w and 0 <= rpy < disp_h:
            cv2.circle(disp_map, (rpx, rpy), 12, COLOR_ROBOT, -1)
            cv2.circle(disp_map, (rpx, rpy), 12, (255, 255, 255), 2)
            ax = int(rpx + 20 * math.cos(-rt))
            ay = int(rpy + 20 * math.sin(-rt))
            cv2.arrowedLine(disp_map, (rpx, rpy), (ax, ay), (255, 255, 255), 2, tipLength=0.4)

        # Blit to Cairo - keep bytes reference alive while pixbuf uses it
        disp_map_rgb = cv2.cvtColor(disp_map, cv2.COLOR_BGR2RGB)
        map_bytes = disp_map_rgb.tobytes()
        pixbuf = GdkPixbuf.Pixbuf.new_from_data(
            map_bytes, GdkPixbuf.Colorspace.RGB, False, 8,
            disp_w, disp_h, disp_w * 3)
        Gdk.cairo_set_source_pixbuf(cr, pixbuf, disp_x, disp_y)
        cr.paint()

        cr.set_source_rgb(0.55, 0.59, 0.63)
        cr.set_font_size(11)
        cr.move_to(x + 10, y + h - 10)
        cr.show_text(f"Robot: ({rx:.2f}, {ry:.2f}) {math.degrees(rt):.0f}\u00b0")
    else:
        cr.set_source_rgb(0.35, 0.38, 0.42)
        cr.set_font_size(16)
        cr.move_to(x + w // 2 - 80, y + h // 2)
        cr.show_text("Waiting for map...")
