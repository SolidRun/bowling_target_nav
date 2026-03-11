"""SLAM map panel rendering with scan-map consistency diagnostics.

Draws the left-hand panel of the main GUI showing:
  - Colorized occupancy grid (free/occupied/unknown cells)
  - Optional grid lines at 1-meter intervals
  - Laser scan overlay with color-coded scan-map consistency:
      Green  = endpoint matches occupied cell (SLAM correct)
      Red    = endpoint hits free cell (SLAM drift / stale map)
      Orange = endpoint in unmapped area
  - Robot marker with heading arrow
  - Camera FOV cone based on camera yaw and horizontal FOV from settings
  - Navigation target diamond connected to robot by a line
  - Three-line diagnostic footer: pose/map info, topic rates/TF age,
    and scan-map consistency percentage

Supports configurable map rotation (rotates the entire rendered map
including all overlays) via the Settings > Map tab.
"""

import math
import time

import cv2
import numpy as np

_last_map_log = 0.0  # periodic logging

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, GdkPixbuf, GLib

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

    Args:
        cr: Cairo context.
        x: Panel left edge in window coordinates.
        y: Panel top edge in window coordinates.
        w: Panel width in pixels.
        h: Panel height in pixels.
        state: Shared state facade providing sensor, detection, and nav data.
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

    # Map display parameters
    mp = state.detection.get_map_params()
    map_rotation = mp.get('rotation', 0.0)
    robot_size = mp.get('robot_size', 12)
    robot_arrow_len = mp.get('robot_arrow_len', 20)
    show_grid = mp.get('show_grid', True)
    show_laser = mp.get('show_laser', True)
    show_nav_target = mp.get('show_nav_target', True)
    laser_ps = mp.get('laser_point_size', 1)

    # Periodic diagnostic: log what map_panel actually receives
    global _last_map_log
    now = time.time()
    if now - _last_map_log > 5.0:
        _last_map_log = now
        print(f"[MapPanel] map_params={mp}  id(_map_params_dict)={id(mp)}", flush=True)

    content_y = y + 35
    content_h = h - 88  # room for 4 diagnostic lines at bottom

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

        # Compute scale factor accounting for rotation bounding box expansion
        rot_rad = math.radians(map_rotation) if map_rotation != 0.0 else 0.0
        if rot_rad != 0.0:
            abs_cos = abs(math.cos(rot_rad))
            abs_sin = abs(math.sin(rot_rad))
            # Bounding box of rotated rectangle
            rot_bw = mw * abs_cos + mh * abs_sin
            rot_bh = mw * abs_sin + mh * abs_cos
        else:
            rot_bw, rot_bh = float(mw), float(mh)

        avail_w = w - 20
        scale = min(avail_w / rot_bw, content_h / rot_bh)
        # Original (unrotated) scaled map dimensions — used for coordinate mapping
        map_pw = int(mw * scale)
        map_ph = int(mh * scale)

        disp_map = cv2.resize(local_map, (map_pw, map_ph), interpolation=cv2.INTER_NEAREST)

        # Apply map rotation — expand canvas to fit rotated content
        if rot_rad != 0.0:
            disp_w = int(rot_bw * scale)
            disp_h = int(rot_bh * scale)
            center = (map_pw // 2, map_ph // 2)
            M_rot = cv2.getRotationMatrix2D(center, map_rotation, 1.0)
            # Shift to center of expanded canvas
            M_rot[0, 2] += (disp_w - map_pw) / 2
            M_rot[1, 2] += (disp_h - map_ph) / 2
            disp_map = cv2.warpAffine(disp_map, M_rot, (disp_w, disp_h),
                                       borderValue=COLOR_UNKNOWN)
        else:
            disp_w, disp_h = map_pw, map_ph

        disp_x = x + (w - disp_w) // 2
        disp_y = content_y + (content_h - disp_h) // 2

        def _rotate_pt(px, py):
            """Rotate point from unrotated map coords to rotated canvas coords."""
            if rot_rad == 0.0:
                return px, py
            # Map coords are relative to unrotated map origin (0,0)
            # Rotate around unrotated map center, then shift to expanded canvas
            mcx, mcy = map_pw / 2, map_ph / 2
            cos_r = math.cos(rot_rad)
            sin_r = math.sin(rot_rad)
            dx, dy = px - mcx, py - mcy
            nx = mcx + dx * cos_r + dy * sin_r
            ny = mcy - dx * sin_r + dy * cos_r
            # Shift from unrotated map center to expanded canvas center
            nx += (disp_w - map_pw) / 2
            ny += (disp_h - map_ph) / 2
            return int(nx), int(ny)

        # Grid lines
        ppm = int(1.0 / res * scale)
        if show_grid and ppm > 15:
            for gx in range(0, disp_w, ppm):
                cv2.line(disp_map, (gx, 0), (gx, disp_h), COLOR_GRID, 1)
            for gy in range(0, disp_h, ppm):
                cv2.line(disp_map, (0, gy), (disp_w, gy), COLOR_GRID, 1)

        # ---- Laser points with scan-map consistency check ----
        if show_laser and len(lpoints) > 0:
            lp = np.asarray(lpoints, dtype=np.float32)
            cos_t, sin_t = math.cos(rt), math.sin(rt)
            wx = rx + lp[:, 0] * cos_t - lp[:, 1] * sin_t
            wy = ry + lp[:, 0] * sin_t + lp[:, 1] * cos_t

            # Display pixel coordinates in unrotated map space
            dpx = ((wx - ox) / res * scale).astype(np.int32)
            dpy = (map_ph - (wy - oy) / res * scale).astype(np.int32)
            d_valid = (dpx >= 0) & (dpx < map_pw) & (dpy >= 0) & (dpy < map_ph)

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

                # Rotate laser coords from unrotated map space to canvas space
                if rot_rad != 0.0:
                    mcx, mcy = map_pw / 2.0, map_ph / 2.0
                    cos_mr = math.cos(rot_rad)
                    sin_mr = math.sin(rot_rad)
                    dx_l = dpx_b.astype(np.float32) - mcx
                    dy_l = dpy_b.astype(np.float32) - mcy
                    dpx_b = (mcx + dx_l * cos_mr + dy_l * sin_mr + (disp_w - map_pw) / 2).astype(np.int32)
                    dpy_b = (mcy - dx_l * sin_mr + dy_l * cos_mr + (disp_h - map_ph) / 2).astype(np.int32)

                # Clip to canvas bounds and draw color-coded laser points
                ps_range = range(-laser_ps, laser_ps + 1)
                for dy in ps_range:
                    for dx in ps_range:
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
                dpx_o = dpx[only_disp].copy()
                dpy_o = dpy[only_disp].copy()
                if rot_rad != 0.0:
                    mcx, mcy = map_pw / 2.0, map_ph / 2.0
                    cos_mr = math.cos(rot_rad)
                    sin_mr = math.sin(rot_rad)
                    dx_l = dpx_o.astype(np.float32) - mcx
                    dy_l = dpy_o.astype(np.float32) - mcy
                    dpx_o = (mcx + dx_l * cos_mr + dy_l * sin_mr + (disp_w - map_pw) / 2).astype(np.int32)
                    dpy_o = (mcy - dx_l * sin_mr + dy_l * cos_mr + (disp_h - map_ph) / 2).astype(np.int32)
                for dy in range(-laser_ps, laser_ps + 1):
                    for dx in range(-laser_ps, laser_ps + 1):
                        yy = np.clip(dpy_o + dy, 0, disp_h - 1)
                        xx = np.clip(dpx_o + dx, 0, disp_w - 1)
                        disp_map[yy, xx] = COLOR_LASER

        # Robot marker (in unrotated map coords, then rotate)
        rpx = int((rx - ox) / res * scale)
        rpy = int(map_ph - (ry - oy) / res * scale)
        rpx, rpy = _rotate_pt(rpx, rpy)
        if 0 <= rpx < disp_w and 0 <= rpy < disp_h:
            cv2.circle(disp_map, (rpx, rpy), robot_size, COLOR_ROBOT, -1)
            cv2.circle(disp_map, (rpx, rpy), robot_size, (255, 255, 255), 2)
            # Heading arrow rotated with map rotation
            heading = rt + rot_rad
            ax = int(rpx + robot_arrow_len * math.cos(heading))
            ay = int(rpy - robot_arrow_len * math.sin(heading))
            cv2.arrowedLine(disp_map, (rpx, rpy), (ax, ay), (255, 255, 255), 2, tipLength=0.4)

            # Camera FOV cone — uses actual camera yaw from TF settings
            try:
                cam_fov = state.detection.get_camera_fov()
                cam_yaw_rad = math.radians(state.detection.get_camera_yaw())
                fov_half = math.radians(cam_fov / 2.0)
                # Camera direction = robot heading (SLAM + map rotation) + camera yaw
                cam_heading = rt + rot_rad + cam_yaw_rad
                fov_len = 2.0 / res * scale  # 2 meters in pixels
                # Center line
                cx_end = int(rpx + fov_len * math.cos(cam_heading))
                cy_end = int(rpy - fov_len * math.sin(cam_heading))
                cv2.line(disp_map, (rpx, rpy), (cx_end, cy_end), (0, 200, 100), 1)
                # Edge lines
                for sign in (-1, 1):
                    edge_angle = cam_heading + sign * fov_half
                    ex = int(rpx + fov_len * math.cos(edge_angle))
                    ey = int(rpy - fov_len * math.sin(edge_angle))
                    cv2.line(disp_map, (rpx, rpy), (ex, ey), (0, 200, 100), 1)
                # Fill semi-transparent FOV area
                n_pts = 12
                fov_pts = [(rpx, rpy)]
                for i in range(n_pts + 1):
                    a = cam_heading - fov_half + (2 * fov_half) * i / n_pts
                    fx = int(rpx + fov_len * math.cos(a))
                    fy = int(rpy - fov_len * math.sin(a))
                    fov_pts.append((fx, fy))
                overlay = disp_map.copy()
                cv2.fillPoly(overlay, [np.array(fov_pts, dtype=np.int32)], (0, 180, 80))
                cv2.addWeighted(overlay, 0.12, disp_map, 0.88, 0, disp_map)
            except Exception:
                pass

            # Navigation target — uses TF2-computed map-frame coordinates
            # The navigator transforms detection from camera_link -> map via TF2,
            # accounting for camera position, yaw, and robot pose automatically.
            nav_target_map = state.nav.get_nav_target_map()
            if show_nav_target and nav_target_map is not None:
                tx_map, ty_map = nav_target_map
                tpx = int((tx_map - ox) / res * scale)
                tpy = int(map_ph - (ty_map - oy) / res * scale)
                tpx, tpy = _rotate_pt(tpx, tpy)
                if 0 <= tpx < disp_w and 0 <= tpy < disp_h:
                    cv2.line(disp_map, (rpx, rpy), (tpx, tpy), (255, 100, 255), 1)
                    pts = np.array([
                        [tpx, tpy - 8], [tpx + 8, tpy],
                        [tpx, tpy + 8], [tpx - 8, tpy]
                    ], dtype=np.int32)
                    cv2.fillPoly(disp_map, [pts], (255, 0, 255))
                    cv2.polylines(disp_map, [pts], True, (255, 255, 255), 1)

        # Blit to Cairo — new_from_bytes owns the data (no dangling pointer)
        disp_map_rgb = cv2.cvtColor(disp_map, cv2.COLOR_BGR2RGB)
        pixbuf = GdkPixbuf.Pixbuf.new_from_bytes(
            GLib.Bytes.new(disp_map_rgb.tobytes()),
            GdkPixbuf.Colorspace.RGB, False, 8,
            disp_w, disp_h, disp_w * 3)
        Gdk.cairo_set_source_pixbuf(cr, pixbuf, disp_x, disp_y)
        cr.paint()

        # ---- Diagnostic overlay (below map) ----
        cr.set_font_size(11)

        # Line 0: LiDAR distance + angle to detected target
        lidar_at = state.nav.get_lidar_at_target()
        if lidar_at < float('inf'):
            cr.set_source_rgb(0.4, 0.7, 1.0)
            cr.move_to(x + 10, y + h - 53)
            # Also show vision distance for comparison
            nav_target_map = state.nav.get_nav_target_map()
            det_info = ""
            if nav_target_map is not None:
                dx = nav_target_map[0] - rx
                dy = nav_target_map[1] - ry
                map_dist = math.sqrt(dx*dx + dy*dy)
                map_ang = math.degrees(math.atan2(dy, dx) - rt)
                det_info = f"  |  Map: {map_dist:.2f}m {map_ang:.0f}\u00b0"
            cr.show_text(f"LiDAR at target: {lidar_at:.2f}m{det_info}")
        else:
            cr.set_source_rgb(0.45, 0.45, 0.45)
            cr.move_to(x + 10, y + h - 53)
            cr.show_text("LiDAR at target: --")

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
