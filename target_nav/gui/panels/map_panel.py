"""LiDAR radar panel -- left-hand panel of the main GUI window.

Renders a top-down radar view of the robot and its surroundings using
raw LiDAR scan points. The robot is always at the center; LiDAR points
are plotted in robot-relative coordinates so the view rotates with the
robot (ego-centric, not map-fixed).

Coordinate system (robot base_link -> screen):
    Robot X (forward) -> screen Y (up, negative direction)
    Robot Y (left)    -> screen X (left, negative direction)
    So: screen_x = center_x - point_y * ppm
        screen_y = center_y - point_x * ppm
    where ppm = pixels_per_meter = panel_size / (2 * radar_range).

Cairo drawing pipeline (per frame, ~15 FPS):
    1. Background fill + border stroke.
    2. Distance circles (1m increments) + crosshair if show_grid.
    3. LiDAR points: vectorized numpy transform to screen coords, then
       small filled rectangles (faster than arcs).
    4. Robot body rectangle + wheels (scaled from real dimensions in cm).
    5. Camera and LiDAR sensor positions (triangle and circle markers).
    6. Forward arrow from robot front edge.
    7. Navigation target diamond (if active and show_nav_target).
    8. Zoom +/- buttons (top-right corner, clickable from MainGUI).
    9. Footer: robot pose and scan diagnostics.

All appearance settings (range, point size, grid, etc.) come from the
Settings > Radar tab via state.detection.get_map_params().

Runs in: GUI process (Process 0) on the GTK main thread, called from
``MainGUI.on_draw()``.

Key function:
    draw_map_panel() -- main entry point called by MainGUI.on_draw().
"""

import math

from target_nav.utils.log import get_logger

logger = get_logger('gui.map_panel')


def draw_map_panel(cr, x, y, w, h, state):
    """Draw LiDAR radar panel with robot, scan points, and nav target.

    All sizes and toggles are read from state.detection.get_map_params()
    so the user can adjust them in Settings > Radar.

    Args:
        cr: Cairo context to draw on.
        x: Panel left edge in window coordinates.
        y: Panel top edge in window coordinates.
        w: Panel width in pixels.
        h: Panel height in pixels.
        state: SharedState facade.
    """
    # Background
    cr.set_source_rgb(0.05, 0.05, 0.08)
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
    cr.show_text("LiDAR Radar")

    # Read state
    rx, ry, rt = state.sensors.get_robot_pose()
    lpoints, sc, _ = state.sensors.get_laser()
    diag = state.sensors.get_diagnostics()

    # Read radar settings from Settings > Radar tab
    mp = state.detection.get_map_params()
    radar_range = mp.get('radar_range', 5)
    point_size = max(1, mp.get('laser_point_size', 2))
    arrow_len = mp.get('robot_arrow_len', 20)
    show_grid = mp.get('show_grid', True)
    show_nav_target = mp.get('show_nav_target', True)

    content_y = y + 35
    content_h = h - 70

    # Radar geometry: fit a square radar into the panel, compute pixels-per-meter.
    # ppm converts meters to pixels. Robot sits at (cx, cy) = panel center.
    size = min(w - 20, content_h)
    ppm = size / (2 * radar_range)
    cx = x + w / 2
    cy = content_y + content_h / 2

    # Clip to panel
    cr.save()
    cr.rectangle(x, content_y, w, content_h)
    cr.clip()

    # Distance circles
    if show_grid:
        cr.set_source_rgba(0.2, 0.25, 0.3, 0.5)
        cr.set_line_width(0.5)
        for r in range(1, radar_range + 1):
            cr.arc(cx, cy, r * ppm, 0, 2 * math.pi)
            cr.stroke()

        # Grid labels
        cr.set_source_rgba(0.4, 0.5, 0.6, 0.7)
        cr.set_font_size(10)
        for r in range(1, radar_range + 1):
            cr.move_to(cx + 3, cy - r * ppm + 12)
            cr.show_text(f"{r}m")

    # Crosshair
    cr.set_source_rgba(0.3, 0.4, 0.3, 0.3)
    cr.set_line_width(0.5)
    cr.move_to(cx, content_y)
    cr.line_to(cx, content_y + content_h)
    cr.stroke()
    cr.move_to(x, cy)
    cr.line_to(x + w, cy)
    cr.stroke()

    # Draw LiDAR points (vectorized coordinate math).
    # Robot-frame: X=forward, Y=left -> Screen: sx = cx - Y*ppm, sy = cy - X*ppm.
    # Points inside the robot body footprint are filtered out — they are either
    # self-reflections or objects physically overlapping the chassis (the LiDAR
    # sits inside the body so rear-facing beams pass through the footprint).
    robot_dims = state.detection.get_robot_dims()
    hl = robot_dims['length'] / 2.0  # half-length (front/rear extent)
    hw = robot_dims['width'] / 2.0   # half-width (left/right extent)
    if len(lpoints) > 0:
        import numpy as np
        pts = np.asarray(lpoints, dtype=np.float32)
        # Filter out points inside robot body footprint
        outside_body = (np.abs(pts[:, 0]) > hl) | (np.abs(pts[:, 1]) > hw)
        pts = pts[outside_body]
        sx = cx - pts[:, 1] * ppm   # Y (left) -> screen X (left)
        sy = cy - pts[:, 0] * ppm   # X (forward) -> screen Y (up)
        # Filter to visible area
        vis = (sx > x) & (sx < x + w) & (sy > content_y) & (sy < content_y + content_h)
        sx = sx[vis]
        sy = sy[vis]
        ps = point_size
        ps2 = ps * 2
        cr.set_source_rgb(0.0, 1.0, 0.6)
        for i in range(len(sx)):
            cr.rectangle(sx[i] - ps, sy[i] - ps, ps2, ps2)
        cr.fill()

    # ── Robot body (true-to-scale from physical dimensions) ──
    # robot_dims, hl, hw already read above for LiDAR body filter
    robot_len_px = robot_dims['length'] * ppm  # forward/backward (X)
    robot_wid_px = robot_dims['width'] * ppm   # left/right (Y)

    # Body rectangle (centered at robot base_link = screen center)
    cr.set_source_rgba(0.2, 0.6, 0.3, 0.4)
    cr.rectangle(cx - robot_wid_px / 2, cy - robot_len_px / 2,
                 robot_wid_px, robot_len_px)
    cr.fill()
    cr.set_source_rgba(0.3, 0.8, 0.4, 0.7)
    cr.set_line_width(1.5)
    cr.rectangle(cx - robot_wid_px / 2, cy - robot_len_px / 2,
                 robot_wid_px, robot_len_px)
    cr.stroke()

    # Front edge highlight — brighter/thicker so user can clearly see
    # where obstacles start relative to the physical robot front.
    front_y_edge = cy - robot_len_px / 2
    cr.set_source_rgba(0.4, 1.0, 0.5, 0.9)
    cr.set_line_width(2.5)
    cr.move_to(cx - robot_wid_px / 2, front_y_edge)
    cr.line_to(cx + robot_wid_px / 2, front_y_edge)
    cr.stroke()
    # "F" label at front center
    cr.set_source_rgba(0.4, 1.0, 0.5, 0.8)
    cr.set_font_size(9)
    cr.move_to(cx - 3, front_y_edge - 3)
    cr.show_text("F")

    # Wheels (4 corners, physical scale)
    wheel_w = 0.03 * ppm
    wheel_h = 0.06 * ppm
    half_track = robot_wid_px / 2
    half_base = robot_len_px / 2
    cr.set_source_rgba(0.6, 0.6, 0.6, 0.8)
    for wx, wy in [(-1, -1), (1, -1), (-1, 1), (1, 1)]:
        cr.rectangle(
            cx + wx * half_track - wheel_w / 2,
            cy + wy * half_base - wheel_h / 2,
            wheel_w, wheel_h)
    cr.fill()

    # ── Camera FOV cone — shows detection field of view ──
    cam_pos = state.detection.get_camera_pos()
    cam_px = cx - cam_pos['y'] * ppm
    cam_py = cy - cam_pos['x'] * ppm
    cam_fov = state.detection.get_camera_fov()
    cam_yaw = state.detection.get_camera_yaw()

    # Draw FOV wedge lines extending from camera position
    fov_len = min(radar_range * 0.6, 2.0) * ppm
    half_fov = math.radians(cam_fov / 2)
    cam_yaw_rad = math.radians(cam_yaw)
    # Screen coords: forward = -Y direction, left = -X direction
    # Base angle: -pi/2 (screen up), adjusted by camera yaw
    base_angle = -(math.pi / 2) - cam_yaw_rad
    cr.set_source_rgba(0.0, 0.9, 0.3, 0.15)
    cr.set_line_width(1)
    fov_ends = []
    for sign in [-1, 1]:
        angle = base_angle + sign * half_fov
        end_x = cam_px + fov_len * math.cos(angle)
        end_y = cam_py + fov_len * math.sin(angle)
        fov_ends.append((end_x, end_y))
        cr.move_to(cam_px, cam_py)
        cr.line_to(end_x, end_y)
    cr.stroke()
    # Fill the FOV cone area
    cr.set_source_rgba(0.0, 0.9, 0.3, 0.05)
    cr.move_to(cam_px, cam_py)
    cr.line_to(fov_ends[0][0], fov_ends[0][1])
    cr.line_to(fov_ends[1][0], fov_ends[1][1])
    cr.close_path()
    cr.fill()

    # Camera triangle marker (scales with zoom, minimum 5px)
    cs = max(5, 0.02 * ppm)
    cr.set_source_rgba(0.0, 0.9, 0.3, 0.9)
    cr.move_to(cam_px, cam_py - cs * 1.5)
    cr.line_to(cam_px - cs, cam_py + cs)
    cr.line_to(cam_px + cs, cam_py + cs)
    cr.close_path()
    cr.fill()
    # Camera label
    cr.set_font_size(9)
    cr.move_to(cam_px + cs + 2, cam_py + 3)
    cr.show_text("CAM")

    # ── LiDAR marker — red circle with label ──
    lidar_pos = state.detection.get_lidar_pos()
    lid_px = cx - lidar_pos['y'] * ppm
    lid_py = cy - lidar_pos['x'] * ppm
    ls = max(4, 0.015 * ppm)
    cr.set_source_rgba(0.9, 0.2, 0.2, 0.9)
    cr.arc(lid_px, lid_py, ls, 0, 2 * math.pi)
    cr.fill()
    # 360° scan ring (faint)
    cr.set_source_rgba(0.9, 0.2, 0.2, 0.15)
    cr.set_line_width(0.5)
    cr.arc(lid_px, lid_py, ls * 3, 0, 2 * math.pi)
    cr.stroke()
    # LiDAR label
    cr.set_source_rgba(0.9, 0.2, 0.2, 0.9)
    cr.set_font_size(9)
    cr.move_to(lid_px + ls + 2, lid_py + 3)
    cr.show_text("LDR")

    # ── Offset lines — dashed lines from center to each sensor ──
    cr.set_dash([3, 3])
    cr.set_line_width(0.8)
    # Only draw if sensor is offset from center (avoid clutter)
    cam_offset = abs(cam_pos['x']) + abs(cam_pos['y'])
    if cam_offset > 0.01:
        cr.set_source_rgba(0.0, 0.9, 0.3, 0.3)
        cr.move_to(cx, cy)
        cr.line_to(cam_px, cam_py)
        cr.stroke()
    lidar_offset = abs(lidar_pos['x']) + abs(lidar_pos['y'])
    if lidar_offset > 0.01:
        cr.set_source_rgba(0.9, 0.2, 0.2, 0.3)
        cr.move_to(cx, cy)
        cr.line_to(lid_px, lid_py)
        cr.stroke()
    cr.set_dash([])

    # ── Center crosshair dot (base_link origin) ──
    cr.set_source_rgba(1.0, 1.0, 1.0, 0.6)
    cr.arc(cx, cy, 2, 0, 2 * math.pi)
    cr.fill()

    # ── Forward arrow (from robot front edge) ──
    cr.set_source_rgb(1.0, 1.0, 0.3)
    cr.set_line_width(2)
    cr.move_to(cx, front_y_edge)
    cr.line_to(cx, front_y_edge - arrow_len)
    cr.stroke()
    cr.move_to(cx - 5, front_y_edge - arrow_len + 5)
    cr.line_to(cx, front_y_edge - arrow_len)
    cr.line_to(cx + 5, front_y_edge - arrow_len + 5)
    cr.stroke()

    # Navigation target diamond (robot-relative, updated every tick)
    target_bl = state.nav.get_target_robot_xy()
    if show_nav_target and target_bl is not None:
        tx, ty = target_bl  # x=forward, y=left in base_link
        tpx = cx - ty * ppm   # Y→screen X (left=left)
        tpy = cy - tx * ppm   # X→screen Y (forward=up)
        # Diamond marker
        cr.set_source_rgb(1.0, 0.2, 0.2)
        cr.move_to(tpx, tpy - 8)
        cr.line_to(tpx + 6, tpy)
        cr.line_to(tpx, tpy + 8)
        cr.line_to(tpx - 6, tpy)
        cr.close_path()
        cr.fill()
        # Distance label — show from camera position (matches camera panel)
        cam_pos = state.detection.get_camera_pos()
        cam_dx = tx - cam_pos['x']
        cam_dy = ty - cam_pos['y']
        dist = math.sqrt(cam_dx * cam_dx + cam_dy * cam_dy)
        cr.set_source_rgb(1.0, 0.4, 0.4)
        cr.set_font_size(10)
        cr.move_to(tpx + 8, tpy + 4)
        cr.show_text(f"{dist:.2f}m")

    cr.restore()

    # Zoom buttons (top-right of radar panel)
    btn_size = 28
    btn_x = x + w - btn_size - 8
    btn_y1 = content_y + 4          # + button
    btn_y2 = btn_y1 + btn_size + 4  # - button

    for by, label in [(btn_y1, "+"), (btn_y2, "-")]:
        cr.set_source_rgba(0.15, 0.2, 0.25, 0.8)
        cr.rectangle(btn_x, by, btn_size, btn_size)
        cr.fill()
        cr.set_source_rgba(0.3, 0.4, 0.5, 0.8)
        cr.set_line_width(1)
        cr.rectangle(btn_x, by, btn_size, btn_size)
        cr.stroke()
        cr.set_source_rgb(0.8, 0.9, 1.0)
        cr.set_font_size(18)
        cr.move_to(btn_x + 8, by + 21)
        cr.show_text(label)

    # Diagnostics footer
    footer_y = content_y + content_h + 2
    cr.set_source_rgb(0.0, 0.7, 0.3)
    cr.set_font_size(11)
    cr.move_to(x + 10, footer_y + 12)
    cr.show_text(f"Robot: ({rx:.2f}, {ry:.2f}) {math.degrees(rt):.0f}\u00b0")

    cr.set_source_rgb(0.0, 0.8, 1.0)
    cr.move_to(x + 10, footer_y + 26)
    scan_hz = diag.get('scan_hz', 0.0) if diag else 0.0
    cr.show_text(f"Scan: {scan_hz:.1f}Hz ({len(lpoints)}pts)  |  Range: {radar_range}m")
