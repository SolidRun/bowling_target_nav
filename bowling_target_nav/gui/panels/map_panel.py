"""SLAM map panel rendering with scan-map consistency diagnostics."""

import math
import time

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

# Laser consistency colors (BGR)
COLOR_MATCH = (0, 255, 0)       # Green: laser hits occupied cell (good)
COLOR_MISMATCH = (0, 0, 255)    # Red: laser hits free cell (map is wrong)
COLOR_UNMAPPED = (0, 180, 255)  # Orange: laser hits unknown cell (not mapped yet)


def draw_map_panel(cr, x, y, w, h, state):
    """Draw SLAM map panel with robot, laser, and navigation target overlay.

    Laser points are color-coded by scan-map consistency:
      Green  = laser endpoint matches occupied cell (SLAM correct)
      Red    = laser endpoint hits free cell (SLAM drift / stale map)
      Orange = laser endpoint in unmapped area
    """
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
    lpoints, sc, _ = state.sensors.get_laser()
    diag = state.sensors.get_diagnostics()

    content_y = y + 35
    content_h = h - 75  # more room for diagnostics at bottom

    # Diagnostic defaults
    consistency = 0.0
    n_match = 0
    n_free_miss = 0
    n_unknown = 0
    n_total = 0

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

        # ---- Laser points with scan-map consistency check ----
        if len(lpoints) > 0:
            lp = np.asarray(lpoints, dtype=np.float32)
            cos_t, sin_t = math.cos(rt), math.sin(rt)
            wx = rx + lp[:, 0] * cos_t - lp[:, 1] * sin_t
            wy = ry + lp[:, 0] * sin_t + lp[:, 1] * cos_t

            # Display pixel coordinates (scaled map)
            dpx = ((wx - ox) / res * scale).astype(np.int32)
            dpy = (disp_h - (wy - oy) / res * scale).astype(np.int32)
            d_valid = (dpx >= 0) & (dpx < disp_w) & (dpy >= 0) & (dpy < disp_h)

            # Original map pixel coordinates (for consistency check)
            # The stored map was cv2.flip(img, 0), so row 0 = top = y_max
            mx = np.floor((wx - ox) / res).astype(np.int32)
            my = (mh - 1 - np.floor((wy - oy) / res)).astype(np.int32)
            m_valid = (mx >= 0) & (mx < mw) & (my >= 0) & (my < mh)

            both = d_valid & m_valid

            if np.any(both):
                mx_b = mx[both]
                my_b = my[both]
                dpx_b = dpx[both]
                dpy_b = dpy[both]

                # Check cell type via green channel (unique per cell type)
                # COLOR_OCCUPIED G=200, COLOR_FREE G=50, COLOR_UNKNOWN G=30
                cell_g = local_map[my_b, mx_b, 1]
                is_occ = cell_g == 200
                is_unk = cell_g == 30
                is_free = ~is_occ & ~is_unk

                n_total = len(mx_b)
                n_match = int(np.sum(is_occ))
                n_unknown = int(np.sum(is_unk))
                n_free_miss = int(np.sum(is_free))
                n_known = n_match + n_free_miss
                consistency = n_match / max(n_known, 1)

                # Draw color-coded laser points (3x3 pixels each)
                for dy in range(-1, 2):
                    for dx in range(-1, 2):
                        if np.any(is_occ):
                            yy = np.clip(dpy_b[is_occ] + dy, 0, disp_h - 1)
                            xx = np.clip(dpx_b[is_occ] + dx, 0, disp_w - 1)
                            disp_map[yy, xx] = COLOR_MATCH
                        if np.any(is_free):
                            yy = np.clip(dpy_b[is_free] + dy, 0, disp_h - 1)
                            xx = np.clip(dpx_b[is_free] + dx, 0, disp_w - 1)
                            disp_map[yy, xx] = COLOR_MISMATCH
                        if np.any(is_unk):
                            yy = np.clip(dpy_b[is_unk] + dy, 0, disp_h - 1)
                            xx = np.clip(dpx_b[is_unk] + dx, 0, disp_w - 1)
                            disp_map[yy, xx] = COLOR_UNMAPPED

            # Points only in display bounds (outside original map) - draw default
            only_disp = d_valid & ~m_valid
            if np.any(only_disp):
                dpx_o = dpx[only_disp]
                dpy_o = dpy[only_disp]
                for dy in range(-1, 2):
                    for dx in range(-1, 2):
                        yy = np.clip(dpy_o + dy, 0, disp_h - 1)
                        xx = np.clip(dpx_o + dx, 0, disp_w - 1)
                        disp_map[yy, xx] = COLOR_LASER

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

        # ---- Diagnostic overlay (below map) ----
        cr.set_font_size(11)

        # Line 1: Robot pose + map info
        cr.set_source_rgb(0.55, 0.59, 0.63)
        cr.move_to(x + 10, y + h - 40)
        cr.show_text(
            f"Robot: ({rx:.2f}, {ry:.2f}) {math.degrees(rt):.0f}\u00b0"
            f"  |  Map: {mw}x{mh} updates:{mc}")

        # Line 2: Topic rates and TF age
        scan_hz = diag.get('scan_hz', 0)
        map_hz = diag.get('map_hz', 0)
        tf_age = diag.get('tf_age', -1)
        scan_age = diag.get('scan_age', -1)

        # Color TF age: green if fresh, yellow if stale, red if very stale
        if 0 <= tf_age < 0.5:
            cr.set_source_rgb(0.0, 0.8, 0.0)
        elif 0 <= tf_age < 2.0:
            cr.set_source_rgb(0.8, 0.7, 0.0)
        else:
            cr.set_source_rgb(0.8, 0.0, 0.0)
        cr.move_to(x + 10, y + h - 26)
        tf_str = f"{tf_age:.1f}s" if tf_age >= 0 else "N/A"
        cr.show_text(
            f"Scan: {scan_hz:.1f}Hz ({diag.get('n_points', 0)}pts)"
            f"  |  Map: {map_hz:.1f}Hz"
            f"  |  TF age: {tf_str}")

        # Line 3: Scan-map consistency
        if n_total > 0:
            if consistency > 0.6:
                cr.set_source_rgb(0.0, 0.85, 0.0)
            elif consistency > 0.3:
                cr.set_source_rgb(0.85, 0.7, 0.0)
            else:
                cr.set_source_rgb(0.85, 0.0, 0.0)
            cr.move_to(x + 10, y + h - 12)
            cr.show_text(
                f"Map match: {consistency * 100:.0f}%"
                f" ({n_match} ok / {n_free_miss} miss / {n_unknown} unmapped"
                f" of {n_total})")
        else:
            cr.set_source_rgb(0.45, 0.45, 0.45)
            cr.move_to(x + 10, y + h - 12)
            cr.show_text("Map match: -- (no laser data)")

    else:
        cr.set_source_rgb(0.35, 0.38, 0.42)
        cr.set_font_size(16)
        cr.move_to(x + w // 2 - 80, y + h // 2)
        cr.show_text("Waiting for map...")

        # Show scan/TF status even without map
        cr.set_font_size(11)
        scan_hz = diag.get('scan_hz', 0)
        tf_age = diag.get('tf_age', -1)
        if scan_hz > 0 or tf_age >= 0:
            cr.set_source_rgb(0.55, 0.55, 0.55)
            cr.move_to(x + 10, y + h - 12)
            tf_str = f"{tf_age:.1f}s" if tf_age >= 0 else "N/A"
            cr.show_text(
                f"Scan: {scan_hz:.1f}Hz"
                f"  |  TF age: {tf_str}"
                f"  |  Map: not received")
