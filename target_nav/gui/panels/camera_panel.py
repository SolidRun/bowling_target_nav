"""Camera panel renderer -- right-hand panel of the main GUI window.

Renders the camera feed panel using pure Cairo (no OpenCV). The DRP-AI
C++ binary draws all detection overlays (bounding boxes, crosshairs,
distance labels) on the frame before writing it to shared memory.
This module just displays the pre-rendered frame scaled to fit the panel,
plus a nav-state badge and an info bar with detection statistics.

The panel is split vertically:

    Top (video area): Live camera feed scaled to fit the panel via Cairo
        surface scaling with bilinear filtering. A nav-state badge is
        overlaid in the top-right corner (rendered by Python).

    Bottom (info bar, 80px): Three-column display of detection stats:
        Col 1: Camera distance (m) + vision angle.
        Col 2: LiDAR distance (m).
        Col 3: Confidence % (color-coded) + speed + detector mode.

Uses frame caching via Cairo ImageSurface (``_frame_cache``) -- only
converts the numpy BGR frame to a Cairo ARGB32 surface when the frame
identity changes (checked by ``id(frame)``). This avoids a costly
numpy-to-surface conversion on every 10 FPS redraw when the camera
process hasn't produced a new frame since the last draw.

Runs in: GUI process (Process 0) on the GTK main thread, called from
``MainGUI.on_draw()`` at 10 FPS.

Key functions:
    draw_camera_panel() -- main entry point called by MainGUI.
    _frame_to_surface() -- BGR numpy to Cairo ARGB32 ImageSurface.
    _draw_info_bar()    -- renders the stats bar below the video.
"""

import math

import cairo
import numpy as np


# Info bar height (pixels)
_INFO_BAR_H = 80

# Frame rendering cache — skip surface rebuild when frame hasn't changed.
_frame_cache = {
    'frame_id': None,       # id() of the last rendered numpy frame
    'surface': None,        # cached Cairo ImageSurface
}


def _frame_to_surface(frame):
    """Convert a BGR numpy frame to a Cairo ImageSurface (ARGB32).

    The frame comes directly from the C++ DRP-AI binary as BGR (with
    all detection overlays already drawn). Cairo ARGB32 on little-endian
    stores pixels as BGRA in memory, so BGR maps directly — just add
    the alpha channel.

    Args:
        frame: BGR numpy array (H x W x 3, uint8) straight from C++ shm.

    Returns:
        cairo.ImageSurface (FORMAT_ARGB32) backed by the converted array.
    """
    h, w = frame.shape[:2]
    # Create a proper Cairo-owned surface (no external buffer management)
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, w, h)
    # Get a writable memoryview into the surface's own buffer
    buf = surface.get_data()
    # BGR frame -> BGRA. Cairo ARGB32 on little-endian stores as B,G,R,A bytes.
    argb = np.empty((h, w, 4), dtype=np.uint8)
    argb[:, :, 0] = frame[:, :, 0]  # B
    argb[:, :, 1] = frame[:, :, 1]  # G
    argb[:, :, 2] = frame[:, :, 2]  # R
    argb[:, :, 3] = 255              # A (opaque)
    # Copy into Cairo's own memory (safe, no dangling pointer)
    stride = surface.get_stride()
    src = argb.tobytes()
    expected = stride * h
    if len(src) == expected:
        buf[:] = src
    else:
        # Handle stride padding if needed
        for row in range(h):
            buf[row * stride:row * stride + w * 4] = src[row * w * 4:(row + 1) * w * 4]
    surface.mark_dirty()
    return surface


def draw_camera_panel(cr, x, y, w, h, state):
    """Render the camera feed panel using pure Cairo (no OpenCV).

    The DRP-AI C++ binary draws all detection overlays (bounding boxes,
    crosshairs, distance labels) on the frame before writing it to
    shared memory. This function just displays the pre-rendered frame
    scaled to fit the panel, plus a nav-state badge and info bar.

    Uses frame caching via Cairo ImageSurface -- only converts the
    numpy BGR frame when it changes (checked by id(frame)).

    Args:
        cr: Cairo context to draw on.
        x: Panel top-left corner X in window coordinates.
        y: Panel top-left corner Y in window coordinates.
        w: Panel width in pixels.
        h: Panel height in pixels.
        state: SharedState facade for detection and nav data.
    """
    # Panel background
    cr.set_source_rgb(0.086, 0.106, 0.133)
    cr.rectangle(x, y, w, h)
    cr.fill()

    frame, detections, info, det_age = state.detection.get_camera()

    # Layout: video on top, info bar at bottom
    info_bar_y = y + h - _INFO_BAR_H
    video_h = h - _INFO_BAR_H - 4  # 4px gap

    if frame is not None:
        fh, fw = frame.shape[:2]

        # Fit frame to panel with uniform scaling (letterboxed, 4px margin)
        scale = min((w - 8) / fw, video_h / fh)
        disp_w = int(fw * scale)
        disp_h = int(fh * scale)
        disp_x = x + (w - disp_w) // 2
        disp_y = y + (video_h - disp_h) // 2

        # Cache: reuse Cairo surface if the numpy frame object hasn't changed.
        # The camera process produces new frame objects; id() changes only
        # when a new numpy array is allocated. This skips the expensive
        # BGR-to-ARGB32 conversion when the 10 FPS GUI redraws between
        # camera frames (camera runs at ~15 FPS depending on DRP-AI load).
        global _frame_cache
        cur_frame_id = id(frame)

        if (cur_frame_id == _frame_cache['frame_id'] and
                _frame_cache['surface'] is not None):
            surface = _frame_cache['surface']
        else:
            surface = _frame_to_surface(frame)
            _frame_cache['frame_id'] = cur_frame_id
            _frame_cache['surface'] = surface

        # Paint the cached Cairo surface scaled to fit the panel.
        # Cairo handles the scaling via cr.scale(); bilinear filtering
        # smooths the upscaled image on the V2N's 1024x600 display.
        cr.save()
        cr.translate(disp_x, disp_y)
        cr.scale(scale, scale)
        cr.set_source_surface(surface, 0, 0)
        cr.get_source().set_filter(cairo.FILTER_BILINEAR)
        cr.paint()

        # Draw bbox overlay for filtered detections (only targets that
        # passed the shape filter — no false positives shown).
        # Green = normal detection, yellow = clipped (bbox truncated at edge).
        if detections and det_age < state.detection.get_detect_expiry():
            for det in detections:
                x1, y1, x2, y2 = det.bbox
                if det.bbox_clipped:
                    cr.set_source_rgba(1.0, 0.8, 0.0, 0.8)  # yellow for clipped
                else:
                    cr.set_source_rgba(0.0, 0.9, 0.2, 0.8)  # green for normal
                cr.set_line_width(2)
                cr.rectangle(x1, y1, x2 - x1, y2 - y1)
                cr.stroke()
                # Label
                cr.set_font_size(12)
                clip_tag = " CLIP" if det.bbox_clipped else ""
                label = f"{det.confidence:.0%} {det.distance:.2f}m{clip_tag}"
                cr.move_to(x1, y1 - 4)
                cr.show_text(label)
                # Crosshair at bbox center
                cx_det = (x1 + x2) / 2
                cy_det = (y1 + y2) / 2
                r = 12
                cr.set_source_rgba(0.0, 0.9, 0.2, 0.6)
                cr.arc(cx_det, cy_det, r, 0, 2 * math.pi)
                cr.stroke()
                cr.move_to(cx_det - r - 4, cy_det)
                cr.line_to(cx_det + r + 4, cy_det)
                cr.stroke()
                cr.move_to(cx_det, cy_det - r - 4)
                cr.line_to(cx_det, cy_det + r + 4)
                cr.stroke()

        cr.restore()

        # Best detection data
        best = None
        cam_dist = 0.0
        cam_angle = 0.0
        confidence = 0.0
        lidar_dist = float('inf')
        clipped = False

        if detections and det_age < state.detection.get_detect_expiry():
            best = min(detections, key=lambda d: d.distance)
            cam_dist = best.distance
            cam_angle = best.angle
            confidence = best.confidence
            clipped = best.bbox_clipped
            lidar_dist = state.nav.get_lidar_at_target()

        # Nav state badge (top-right of video)
        nav_state, _ = state.nav.get_nav_state()
        badge_colors = {
            "NAVIGATING": (0.137, 0.533, 0.212),
            "SEARCHING": (0.886, 0.686, 0.0),
            "BLIND_APPROACH": (0.902, 0.557, 0.149),
            "SPIRAL_SEARCH": (0.878, 0.565, 0.251),
            "ARRIVED": (0.122, 0.435, 0.918),
            "IDLE": (0.35, 0.38, 0.42),
            "ERROR": (0.855, 0.212, 0.200),
        }
        bc = badge_colors.get(nav_state, (0.35, 0.38, 0.42))
        badge_text = nav_state.replace("_", " ")
        cr.set_font_size(12)
        extents = cr.text_extents(badge_text)
        bw = extents.width + 14
        badge_bh = 22
        badge_x = disp_x + disp_w - 10 - bw
        badge_y = disp_y + 8

        cr.set_source_rgba(bc[0], bc[1], bc[2], 0.85)
        cr.rectangle(badge_x, badge_y, bw, badge_bh)
        cr.fill()
        cr.set_source_rgb(1.0, 1.0, 1.0)
        cr.move_to(badge_x + 7, badge_y + 16)
        cr.show_text(badge_text)

        # Info bar below video
        _draw_info_bar(cr, x, info_bar_y, w, _INFO_BAR_H,
                       best, cam_dist, cam_angle, confidence,
                       lidar_dist, clipped, nav_state, state)

    else:
        det_mode = state.detection.get_detector_mode()
        if "Initializing" in det_mode or "Stream" in det_mode:
            cr.set_source_rgb(0.3, 0.5, 0.6)
            cr.set_font_size(14)
            cr.move_to(x + w // 2 - 80, y + h // 2)
            cr.show_text("Starting DRP-AI camera...")
        else:
            cr.set_source_rgb(0.35, 0.38, 0.42)
            cr.set_font_size(16)
            cr.move_to(x + w // 2 - 70, y + h // 2)
            cr.show_text("No camera feed")

        # Empty info bar
        _draw_info_bar(cr, x, info_bar_y, w, _INFO_BAR_H,
                       None, 0, 0, 0, float('inf'), False, "IDLE", state)


def _draw_info_bar(cr, x, y, w, h, best, cam_dist, cam_angle,
                   confidence, lidar_dist, clipped, nav_state, state):
    """Render the info bar with large readable detection stats below the video.

    Layout (3 equal-width columns):
        Col 1: Camera distance (large) + vision angle (small).
        Col 2: LiDAR distance (large) + 'lidar' label (small).
        Col 3: Confidence % (large, color-coded) + speed/omega (small).
        Bottom row: Detector mode string.

    Grayed out when no target is detected (``best is None``).

    Args:
        cr: Cairo context to draw on.
        x: Info bar left edge in window coordinates.
        y: Info bar top edge in window coordinates.
        w: Info bar width in pixels.
        h: Info bar height in pixels (typically 80).
        best: Closest Detection object, or None.
        cam_dist: Camera-estimated distance in meters.
        cam_angle: Camera-estimated angle in radians.
        confidence: Detection confidence (0.0-1.0).
        lidar_dist: LiDAR distance to target in meters (inf if unavailable).
        clipped: True if the detection bbox is clipped at frame edge.
        nav_state: Current navigation state string.
        state: SharedState facade for reading velocity and detector mode.
    """
    # Background
    cr.set_source_rgb(0.11, 0.13, 0.16)
    cr.rectangle(x, y, w, h)
    cr.fill()

    # Top separator line
    cr.set_source_rgb(0.25, 0.28, 0.32)
    cr.set_line_width(1)
    cr.move_to(x, y)
    cr.line_to(x + w, y)
    cr.stroke()

    col_w = w / 3.0
    pad = 10
    has_target = best is not None

    # Reference point label
    try:
        ref_pt = state.detection.get_ref_point()
    except Exception:
        ref_pt = "front"

    # -- Column 1: Camera Distance (adjusted to ref point) --
    col_x = x + pad
    if has_target:
        cr.set_source_rgb(0.2, 1.0, 0.5)
        cr.set_font_size(28)
        cr.move_to(col_x, y + 32)
        cr.show_text(f"{cam_dist:.2f}m")

        cr.set_source_rgba(0.7, 0.8, 0.7, 0.7)
        cr.set_font_size(11)
        cr.move_to(col_x, y + 48)
        angle_deg = math.degrees(cam_angle)
        cr.show_text(f"vision  {angle_deg:+.0f}\u00b0")
    else:
        cr.set_source_rgba(0.4, 0.45, 0.5, 0.8)
        cr.set_font_size(28)
        cr.move_to(col_x, y + 32)
        cr.show_text("--.-m")

        cr.set_source_rgba(0.4, 0.45, 0.5, 0.5)
        cr.set_font_size(11)
        cr.move_to(col_x, y + 48)
        cr.show_text("vision")

    # -- Column 2: LiDAR Distance --
    col_x = x + col_w + pad
    if has_target and lidar_dist < 100.0:
        cr.set_source_rgb(0.4, 0.75, 1.0)
        cr.set_font_size(28)
        cr.move_to(col_x, y + 32)
        cr.show_text(f"{lidar_dist:.2f}m")
        cr.set_source_rgba(0.5, 0.7, 0.9, 0.7)
        cr.set_font_size(11)
        cr.move_to(col_x, y + 48)
        cr.show_text("lidar")
    else:
        cr.set_source_rgba(0.4, 0.45, 0.5, 0.8)
        cr.set_font_size(28)
        cr.move_to(col_x, y + 32)
        cr.show_text("--.-m")
        cr.set_source_rgba(0.4, 0.45, 0.5, 0.5)
        cr.set_font_size(11)
        cr.move_to(col_x, y + 48)
        cr.show_text("sensor")

    # -- Column 3: Confidence + Speed --
    col_x = x + col_w * 2 + pad
    if has_target:
        pct = confidence * 100.0
        # Color: green >70%, yellow 50-70%, red <50%
        if pct >= 70:
            cr.set_source_rgb(0.2, 1.0, 0.5)
        elif pct >= 50:
            cr.set_source_rgb(1.0, 0.85, 0.2)
        else:
            cr.set_source_rgb(1.0, 0.4, 0.3)
        cr.set_font_size(28)
        cr.move_to(col_x, y + 32)
        cr.show_text(f"{pct:.0f}%")

        if clipped:
            cr.set_source_rgba(1.0, 0.3, 0.0, 0.9)
            cr.set_font_size(11)
            cr.move_to(col_x + 60, y + 32)
            cr.show_text("CLIP")
    else:
        cr.set_source_rgba(0.4, 0.45, 0.5, 0.8)
        cr.set_font_size(28)
        cr.move_to(col_x, y + 32)
        cr.show_text("--%")

    # Speed line (bottom of column 3)
    vx, vy, wz = state.nav.get_current_cmd_vel()
    speed = math.sqrt(vx * vx + vy * vy)
    if speed > 0.001 or abs(wz) > 0.01:
        cr.set_source_rgba(0.0, 0.85, 1.0, 0.8)
    else:
        cr.set_source_rgba(0.4, 0.45, 0.5, 0.5)
    cr.set_font_size(11)
    cr.move_to(col_x, y + 48)
    cr.show_text(f"v={speed:.2f}  \u03c9={math.degrees(wz):.0f}\u00b0/s")

    # -- Bottom row: detector mode --
    det_mode = state.detection.get_detector_mode()
    cr.set_source_rgba(0.5, 0.55, 0.6, 0.6)
    cr.set_font_size(10)
    cr.move_to(x + pad, y + h - 8)
    cr.show_text(det_mode)
