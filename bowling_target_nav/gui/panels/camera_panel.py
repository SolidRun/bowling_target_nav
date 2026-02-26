"""Camera panel rendering with detection overlays."""

import math

import cv2

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, GdkPixbuf


def draw_camera_panel(cr, x, y, w, h, state):
    """Draw camera panel with detection overlays, crosshair, state badge, speed."""
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
    cr.set_source_rgb(0.902, 0.557, 0.149)
    cr.set_font_size(15)
    cr.move_to(x + 10, y + 20)
    cr.show_text("Camera + Detection")

    # Detector mode badge (top-right of header)
    det_mode = state.detection.get_detector_mode()
    if "DRP-AI" in det_mode:
        badge_bg = (0.0, 0.6, 0.15)
        badge_fg = (1.0, 1.0, 1.0)
    elif "ONNX" in det_mode:
        badge_bg = (0.75, 0.6, 0.0)
        badge_fg = (1.0, 1.0, 1.0)
    elif det_mode == "Initializing":
        badge_bg = (0.35, 0.38, 0.42)
        badge_fg = (0.8, 0.8, 0.8)
    else:
        badge_bg = (0.5, 0.15, 0.15)
        badge_fg = (1.0, 1.0, 1.0)

    cr.set_font_size(10)
    extents = cr.text_extents(det_mode)
    bw = extents.width + 12
    bh = 16
    bx = x + w - 10 - bw
    by = y + 8
    cr.set_source_rgb(*badge_bg)
    cr.rectangle(bx, by, bw, bh)
    cr.fill()
    cr.set_source_rgb(*badge_fg)
    cr.move_to(bx + 6, by + 12)
    cr.show_text(det_mode)

    frame, detections, info, det_age = state.detection.get_camera()
    nav_state, _ = state.nav.get_nav_state()

    content_y = y + 35
    content_h = h - 55

    if frame is not None:
        fh, fw = frame.shape[:2]
        scale = min((w - 20) / fw, content_h / fh)
        disp_w = int(fw * scale)
        disp_h = int(fh * scale)
        disp_x = x + (w - disp_w) // 2
        disp_y = content_y + (content_h - disp_h) // 2

        disp_frame = cv2.resize(frame, (disp_w, disp_h))

        # Keep reference to bytes data alive while pixbuf uses it -
        # new_from_data does NOT copy the buffer, so GC of the temporary
        # from tobytes() would leave a dangling pointer.
        frame_bytes = disp_frame.tobytes()
        pixbuf = GdkPixbuf.Pixbuf.new_from_data(
            frame_bytes, GdkPixbuf.Colorspace.RGB, False, 8,
            disp_w, disp_h, disp_w * 3)
        Gdk.cairo_set_source_pixbuf(cr, pixbuf, disp_x, disp_y)
        cr.paint()

        # Crosshair on best detection (skip in stream mode — C++ draws it)
        if detections and det_age < 1.5 and not state.detection.get_stream_mode():
            best = min(detections, key=lambda d: d.get('distance', 999))
            bx1, by1, bx2, by2 = best['bbox']
            cx = disp_x + int((bx1 + bx2) / 2.0 * scale)
            cy = disp_y + int((by1 + by2) / 2.0 * scale)
            r = 18

            cr.set_source_rgba(0.0, 1.0, 0.4, 0.8)
            cr.set_line_width(2.0)
            cr.arc(cx, cy, r, 0, 2 * math.pi)
            cr.stroke()
            # Crosshair lines (horizontal and vertical ticks)
            cr.move_to(cx - r - 6, cy)
            cr.line_to(cx - r + 6, cy)
            cr.move_to(cx + r - 6, cy)
            cr.line_to(cx + r + 6, cy)
            cr.move_to(cx, cy - r - 6)
            cr.line_to(cx, cy - r + 6)
            cr.move_to(cx, cy + r - 6)
            cr.line_to(cx, cy + r + 6)
            cr.stroke()

            dist = best.get('distance', 0)
            ang = best.get('angle', 0)
            label = f"{dist:.2f}m  {math.degrees(ang):.0f}\u00b0"
            cr.set_source_rgba(0.0, 1.0, 0.4, 0.9)
            cr.set_font_size(13)
            cr.move_to(cx + r + 8, cy - 4)
            cr.show_text(label)

            # Clipped bbox indicator (distance estimate unreliable)
            if best.get('bbox_clipped', False):
                cr.set_source_rgba(1.0, 0.3, 0.0, 0.9)
                cr.set_font_size(11)
                cr.move_to(cx + r + 8, cy + 10)
                cr.show_text("CLIPPED")

        # Nav state badge
        badge_colors = {
            "NAVIGATING": (0.137, 0.533, 0.212),
            "SEARCHING": (0.886, 0.686, 0.0),
            "BLIND_APPROACH": (0.902, 0.557, 0.149),
            "ARRIVED": (0.122, 0.435, 0.918),
            "IDLE": (0.35, 0.38, 0.42),
            "ERROR": (0.855, 0.212, 0.200),
        }
        bc = badge_colors.get(nav_state, (0.35, 0.38, 0.42))
        badge_text = nav_state.replace("_", " ")
        cr.set_font_size(11)
        extents = cr.text_extents(badge_text)
        bw = extents.width + 14
        bh = 20
        badge_x = disp_x + disp_w - 10 - bw
        badge_y = disp_y + 8

        cr.set_source_rgba(bc[0], bc[1], bc[2], 0.85)
        cr.rectangle(badge_x, badge_y, bw, bh)
        cr.fill()
        cr.set_source_rgb(1.0, 1.0, 1.0)
        cr.move_to(badge_x + 7, badge_y + 14)
        cr.show_text(badge_text)

        # Speed indicator
        vx, vy, wz = state.nav.get_current_cmd_vel()
        speed = math.sqrt(vx * vx + vy * vy)
        if speed > 0.001 or abs(wz) > 0.01:
            speed_text = f"v={speed:.2f} m/s  \u03c9={math.degrees(wz):.0f}\u00b0/s"
            cr.set_source_rgba(0.0, 0.9, 1.0, 0.8)
            cr.set_font_size(12)
            cr.move_to(disp_x + 8, disp_y + disp_h - 8)
            cr.show_text(speed_text)
    else:
        cr.set_source_rgb(0.35, 0.38, 0.42)
        cr.set_font_size(16)
        cr.move_to(x + w // 2 - 70, y + h // 2)
        cr.show_text("No camera feed")

    cr.set_source_rgb(0.788, 0.820, 0.855)
    cr.set_font_size(12)
    cr.move_to(x + 10, y + h - 10)
    cr.show_text(f"Detection: {info}")
