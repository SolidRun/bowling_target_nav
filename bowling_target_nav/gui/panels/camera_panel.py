"""Camera panel rendering with detection overlays.

Draws the right-hand panel of the main GUI showing:
  - Live camera feed (BGR frame from shared_state, resized to fit panel)
  - Crosshair overlay on the best (closest) detection with distance/angle label
  - "CLIPPED" warning when a bounding box touches the frame edge
  - Navigation state badge (color-coded by NAVIGATING/SEARCHING/ARRIVED/etc.)
  - Speed indicator showing linear velocity and angular rate
  - Detection info footer (detector mode + current target status)

If no camera frame is available, displays a "No camera feed" placeholder.
All rendering uses Cairo primitives and GdkPixbuf for frame blitting.
"""

import math

import cv2

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, GdkPixbuf, GLib


def draw_camera_panel(cr, x, y, w, h, state):
    """Draw camera panel with detection overlays.

    Args:
        cr: Cairo context.
        x: Panel left edge in window coordinates.
        y: Panel top edge in window coordinates.
        w: Panel width in pixels.
        h: Panel height in pixels.
        state: Shared state facade providing detection and nav data.
    """
    # Panel background
    cr.set_source_rgb(0.086, 0.106, 0.133)
    cr.rectangle(x, y, w, h)
    cr.fill()

    frame, detections, info, det_age = state.detection.get_camera()

    if frame is not None:
        fh, fw = frame.shape[:2]

        # Leave room for header/footer
        content_y = y + 35
        content_h = h - 55
        scale = min((w - 20) / fw, content_h / fh)
        disp_w = int(fw * scale)
        disp_h = int(fh * scale)
        disp_x = x + (w - disp_w) // 2
        disp_y = content_y + (content_h - disp_h) // 2

        disp_frame = cv2.resize(frame, (disp_w, disp_h))

        pixbuf = GdkPixbuf.Pixbuf.new_from_bytes(
            GLib.Bytes.new(disp_frame.tobytes()),
            GdkPixbuf.Colorspace.RGB, False, 8,
            disp_w, disp_h, disp_w * 3)
        Gdk.cairo_set_source_pixbuf(cr, pixbuf, disp_x, disp_y)
        cr.paint()

        # Header
        cr.set_source_rgb(0.902, 0.557, 0.149)
        cr.set_font_size(15)
        cr.move_to(x + 10, y + 20)
        cr.show_text("Camera + Detection")

        # Crosshair on best detection
        if detections and det_age < 1.5:
            best = min(detections, key=lambda d: d.get('distance', 999))
            bx1, by1, bx2, by2 = best['bbox']
            cx = disp_x + int((bx1 + bx2) / 2.0 * scale)
            cy = disp_y + int((by1 + by2) / 2.0 * scale)
            r = 18

            cr.set_source_rgba(0.0, 1.0, 0.4, 0.8)
            cr.set_line_width(2.0)
            cr.arc(cx, cy, r, 0, 2 * math.pi)
            cr.stroke()
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
            label = f"Cam: {dist:.2f}m  {math.degrees(ang):.0f}\u00b0"
            cr.set_source_rgba(0.0, 1.0, 0.4, 0.9)
            cr.set_font_size(13)
            cr.move_to(cx + r + 8, cy - 4)
            cr.show_text(label)

            # LiDAR distance at target angle for comparison
            lidar_dist = state.nav.get_lidar_at_target()
            if lidar_dist < float('inf'):
                cr.set_source_rgba(0.4, 0.7, 1.0, 0.9)
                cr.set_font_size(12)
                cr.move_to(cx + r + 8, cy + 12)
                cr.show_text(f"LiDAR: {lidar_dist:.2f}m")
                next_y = cy + 26
            else:
                next_y = cy + 10

            if best.get('bbox_clipped', False):
                cr.set_source_rgba(1.0, 0.3, 0.0, 0.9)
                cr.set_font_size(11)
                cr.move_to(cx + r + 8, next_y)
                cr.show_text("CLIPPED")

        # Nav state badge
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

        # Info footer
        cr.set_source_rgb(0.788, 0.820, 0.855)
        cr.set_font_size(12)
        cr.move_to(x + 10, y + h - 10)
        cr.show_text(f"Detection: {info}")
    else:
        cr.set_source_rgb(0.35, 0.38, 0.42)
        cr.set_font_size(16)
        cr.move_to(x + w // 2 - 70, y + h // 2)
        cr.show_text("No camera feed")
