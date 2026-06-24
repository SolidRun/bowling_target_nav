"""Calibration and Mounting sub-tabs for the Sensors page.

Provides two sub-page builders called from DetectTabMixin._build_sensors_page():

    _build_calibration_subpage: Distance calibration, target dims, camera FOV,
                                 distance reference point.
    _build_mounting_subpage:     Robot body dims, LiDAR pos + yaw, camera pos + yaw,
                                 robot diagram.

All changes are written to SettingsStore and auto-saved after debounce.

Runs in: GUI process (Process 0) on the GTK main thread.
"""

import math

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

from target_nav.config import DEFAULT_FRAME_H
from target_nav.utils.log import get_logger

logger = get_logger('gui.settings.setup')


class SetupTabMixin:
    """Calibration and mounting sub-tab builders.

    Auto-calibration workflow (CALIBRATE button):
        1. User places the target at a known distance and enters that
           distance in the "Known dist" spinner.
        2. User clicks CALIBRATE. ``_on_calibrate()`` grabs the current
           best detection from ``state.detection.get_camera()``.
        3. The tallest bounding box height (pixels) is paired with the
           known distance to form a reference pair (ref_height, ref_dist).
        4. This pair is written to SettingsStore via ``set_calibration()``
           and persisted to disk on next auto-save.
        5. The DistanceEstimator uses this pair for all subsequent
           pinhole-model distance estimates (d = ref_dist * ref_h / box_h).

    Manual calibration (APPLY button) lets the user type ref_height and
    ref_dist directly without needing a live detection.
    """

    # ============================================================
    # Calibration sub-tab
    # ============================================================

    def _build_calibration_subpage(self):
        """Build the Calibration sub-tab: distance cal, target dims, FOV, ref point.

        Sections:
            Distance Calibration -- Auto (CALIBRATE) or manual (APPLY) entry of
                the reference pair (box_height_px, known_distance_m) used by
                DistanceEstimator for pinhole-model distance calculation.
            Target + Camera -- Physical target height/width (cm) and camera
                horizontal FOV (degrees).
            Distance Reference -- Which robot reference point (center, front
                edge, camera, or LiDAR) the displayed distances are measured
                from. Changes the sensor offset applied to raw estimates.
        """
        det = self._state.detection
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box.set_margin_top(4)

        # -- Distance calibration --
        box.pack_start(self._make_section("Distance Calibration"), False, False, 0)
        ref_height, ref_dist = det.get_calibration()
        self._cal_label = Gtk.Label(
            label=f"Current: {ref_height:.0f}px at {ref_dist:.2f}m")
        self._cal_label.set_xalign(0)
        self._cal_label.set_margin_start(10)
        box.pack_start(self._cal_label, False, False, 2)

        cal_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        cal_row.set_margin_start(10)
        cal_row.set_margin_end(10)
        cal_row.pack_start(Gtk.Label(label="Known dist:"), False, False, 0)
        self._cal_dist_spin = Gtk.SpinButton.new_with_range(0.1, 5.0, 0.05)
        self._cal_dist_spin.set_value(1.0)
        self._cal_dist_spin.set_digits(2)
        cal_row.pack_start(self._cal_dist_spin, False, False, 0)
        cal_row.pack_start(Gtk.Label(label="m"), False, False, 2)
        cal_btn = Gtk.Button(label="CALIBRATE")
        cal_btn.set_size_request(100, 34)
        cal_btn.connect('clicked', self._on_calibrate)
        cal_row.pack_start(cal_btn, False, False, 4)
        self._cal_result = Gtk.Label(label="")
        cal_row.pack_start(self._cal_result, True, True, 0)
        box.pack_start(cal_row, False, False, 2)

        # Manual entry
        manual_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        manual_row.set_margin_start(10)
        manual_row.set_margin_end(10)
        manual_row.pack_start(Gtk.Label(label="Manual: H"), False, False, 0)
        self._manual_h_spin = Gtk.SpinButton.new_with_range(10, DEFAULT_FRAME_H, 1)
        self._manual_h_spin.set_value(ref_height)
        manual_row.pack_start(self._manual_h_spin, False, False, 0)
        manual_row.pack_start(Gtk.Label(label="px @"), False, False, 4)
        self._manual_d_spin = Gtk.SpinButton.new_with_range(0.1, 5.0, 0.05)
        self._manual_d_spin.set_value(ref_dist)
        self._manual_d_spin.set_digits(2)
        manual_row.pack_start(self._manual_d_spin, False, False, 0)
        manual_row.pack_start(Gtk.Label(label="m"), False, False, 2)
        apply_btn = Gtk.Button(label="APPLY")
        apply_btn.set_size_request(70, 34)
        apply_btn.connect('clicked', self._on_manual_cal)
        manual_row.pack_start(apply_btn, False, False, 4)
        box.pack_start(manual_row, False, False, 2)

        # -- Target + FOV in one row --
        box.pack_start(self._make_section("Target + Camera"), False, False, 0)
        tgt_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        tgt_row.set_margin_start(10)
        target = det.get_target_dims()
        r1, self._sp_tgt_h = self._make_spin_row(
            "Height", 5, 100, 1, target['height'] * 100, 0, "cm")
        r2, self._sp_tgt_w = self._make_spin_row(
            "Width", 2, 50, 1, target['width'] * 100, 0, "cm")
        r3, self._sp_cam_fov = self._make_spin_row(
            "FOV", 30, 120, 1, det.get_camera_fov(), 0, "deg")
        for r in (r1, r2, r3):
            tgt_row.pack_start(r, False, False, 0)
        box.pack_start(tgt_row, False, False, 2)
        for sp in (self._sp_tgt_h, self._sp_tgt_w):
            sp.connect('value-changed', self._on_target_changed)
        self._sp_cam_fov.connect('value-changed', self._on_fov_changed)

        # -- Reference point --
        box.pack_start(self._make_section("Distance Reference"), False, False, 0)
        from target_nav.config import REF_POINTS
        ref_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        ref_row.set_margin_start(10)
        ref_row.pack_start(Gtk.Label(label="Measure from:"), False, False, 0)
        self._ref_combo = Gtk.ComboBoxText()
        ref_labels = {'center': "Center", 'front': "Front edge",
                      'camera': "Camera", 'lidar': "LiDAR"}
        current_ref = det.get_ref_point()
        for i, rp in enumerate(REF_POINTS):
            self._ref_combo.append_text(ref_labels.get(rp, rp))
            if rp == current_ref:
                self._ref_combo.set_active(i)
        self._ref_combo.connect('changed', self._on_ref_point_changed)
        ref_row.pack_start(self._ref_combo, False, False, 0)
        self._ref_info_label = Gtk.Label()
        self._ref_info_label.set_xalign(0)
        ref_row.pack_start(self._ref_info_label, True, True, 6)
        box.pack_start(ref_row, False, False, 2)
        self._update_ref_info()

        box.pack_start(self._make_reset_row(
            self._on_reset_hardware_defaults), False, False, 0)

        return box

    # ============================================================
    # Mounting sub-tab
    # ============================================================

    def _build_mounting_subpage(self):
        """Build the Mounting sub-tab: robot dims, sensor positions, diagram.

        Sections:
            Robot Body -- Length (forward), width (lateral), height in cm.
                Used for radar panel body rendering and obstacle clearance.
            LiDAR -- Forward/lateral/height offsets in cm from robot center,
                plus yaw angle. Defines the TF transform base_link -> lidar.
            Camera -- Forward/lateral/height offsets in cm from robot center,
                plus yaw angle. Defines the TF transform base_link -> camera.
            Robot Diagram -- Live top-down Cairo drawing showing body outline,
                wheels, and sensor positions. Updates as spinbuttons change.

        All positions use robot base_link frame (X=forward, Y=left, Z=up).
        """
        det = self._state.detection
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box.set_margin_top(4)

        # -- Robot body --
        box.pack_start(self._make_section("Robot Body (cm)"), False, False, 0)
        robot = det.get_robot_dims()
        dims_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        dims_row.set_margin_start(10)
        r1, self._sp_robot_len = self._make_spin_row(
            "Length", 10, 60, 1, robot['length'] * 100, 0, "cm")
        r2, self._sp_robot_wid = self._make_spin_row(
            "Width", 10, 60, 1, robot['width'] * 100, 0, "cm")
        r3, self._sp_robot_hgt = self._make_spin_row(
            "Height", 5, 40, 1, robot['height'] * 100, 0, "cm")
        for r in (r1, r2, r3):
            dims_row.pack_start(r, False, False, 0)
        box.pack_start(dims_row, False, False, 2)
        for sp in (self._sp_robot_len, self._sp_robot_wid, self._sp_robot_hgt):
            sp.connect('value-changed', self._on_robot_dims_changed)

        # -- LiDAR position + yaw --
        box.pack_start(self._make_section("LiDAR (cm)"), False, False, 0)
        lidar = det.get_lidar_pos()
        lidar_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        lidar_row.set_margin_start(10)
        r1, self._sp_lidar_x = self._make_spin_row(
            "Fwd", -20, 20, 0.5, lidar['x'] * 100, 1, "")
        r2, self._sp_lidar_y = self._make_spin_row(
            "Lat", -20, 20, 0.5, lidar['y'] * 100, 1, "")
        r3, self._sp_lidar_z = self._make_spin_row(
            "Ht", 0, 30, 0.5, lidar['z'] * 100, 1, "")
        for r in (r1, r2, r3):
            lidar_row.pack_start(r, False, False, 0)
        box.pack_start(lidar_row, False, False, 2)
        for sp in (self._sp_lidar_x, self._sp_lidar_y, self._sp_lidar_z):
            sp.connect('value-changed', self._on_sensor_pos_changed)

        box.pack_start(self._make_angle_row(
            'lidar_yaw', "LiDAR Yaw",
            det.get_lidar_yaw(),
            self._on_lidar_yaw_changed), False, False, 0)

        # -- Camera position + yaw --
        box.pack_start(self._make_section("Camera (cm)"), False, False, 0)
        cam = det.get_camera_pos()
        cam_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        cam_row.set_margin_start(10)
        r1, self._sp_cam_x = self._make_spin_row(
            "Fwd", -20, 20, 0.5, cam['x'] * 100, 1, "")
        r2, self._sp_cam_y = self._make_spin_row(
            "Lat", -20, 20, 0.5, cam['y'] * 100, 1, "")
        r3, self._sp_cam_z = self._make_spin_row(
            "Ht", 0, 30, 0.5, cam['z'] * 100, 1, "")
        for r in (r1, r2, r3):
            cam_row.pack_start(r, False, False, 0)
        box.pack_start(cam_row, False, False, 2)
        for sp in (self._sp_cam_x, self._sp_cam_y, self._sp_cam_z):
            sp.connect('value-changed', self._on_sensor_pos_changed)

        box.pack_start(self._make_angle_row(
            'camera_yaw', "Camera Yaw",
            det.get_camera_yaw(),
            self._on_camera_yaw_changed), False, False, 0)
        
        box.pack_start(self._make_switch_row(  
            'camera_flip_h', "Mirror image",
            det.get_camera_flip_h(),
            lambda v: (self._state.detection.set_camera_flip_h(v),
                    self._schedule_save())), False, False, 0)

        # -- Robot diagram (compact) --
        self._robot_da = Gtk.DrawingArea()
        self._robot_da.set_size_request(-1, 140)
        self._robot_da.connect('draw', self._draw_robot_diagram)
        box.pack_start(self._robot_da, True, True, 2)

        return self._scrolled_page(box)

    # ---- Robot diagram ----

    def _draw_robot_diagram(self, widget, cr):
        """Render top-down robot diagram showing body, wheels, sensors."""
        try:
            alloc = widget.get_allocation()
            W, H = alloc.width, alloc.height
            cr.set_source_rgb(0.12, 0.12, 0.14)
            cr.paint()

            rlen = self._sp_robot_len.get_value()
            rwid = self._sp_robot_wid.get_value()
            lx = self._sp_lidar_x.get_value()
            ly = self._sp_lidar_y.get_value()
            cx = self._sp_cam_x.get_value()
            cy = self._sp_cam_y.get_value()

            max_dim = max(rlen, rwid, 30)
            scale = min((W - 60) / (max_dim * 1.5),
                        (H - 40) / (max_dim * 1.5))
            ox, oy = W / 2, H / 2 + 5

            def to_px(xcm, ycm):
                return ox + ycm * scale, oy - xcm * scale

            cr.set_source_rgba(0.5, 0.8, 1.0, 0.6)
            cr.set_font_size(10)
            cr.move_to(ox - 14, oy - (rlen / 2) * scale - 8)
            cr.show_text("FRONT")

            bx, by = to_px(rlen / 2, rwid / 2)
            bw, bh = rwid * scale, rlen * scale
            cr.set_source_rgba(0.2, 0.3, 0.6, 0.4)
            cr.rectangle(bx, by, bw, bh)
            cr.fill()
            cr.set_source_rgba(0.3, 0.5, 0.9, 0.8)
            cr.set_line_width(1.5)
            cr.rectangle(bx, by, bw, bh)
            cr.stroke()

            for wx, wy in [(rlen/2-2, rwid/2-1), (rlen/2-2, -rwid/2+1),
                           (-rlen/2+2, rwid/2-1), (-rlen/2+2, -rwid/2+1)]:
                wpx, wpy = to_px(wx, wy)
                cr.set_source_rgba(0.4, 0.4, 0.4, 0.9)
                cr.rectangle(wpx-2, wpy-3, 4, 6)
                cr.fill()

            cpx, cpy = to_px(0, 0)
            cr.set_source_rgba(1, 1, 1, 0.4)
            cr.set_line_width(1)
            cr.move_to(cpx-6, cpy); cr.line_to(cpx+6, cpy)
            cr.move_to(cpx, cpy-6); cr.line_to(cpx, cpy+6)
            cr.stroke()

            lpx, lpy = to_px(lx, ly)
            cr.set_source_rgb(1, 0.2, 0.2)
            cr.arc(lpx, lpy, 5, 0, 2*math.pi); cr.fill()
            cr.set_font_size(9)
            cr.set_source_rgb(1, 0.4, 0.4)
            cr.move_to(lpx+7, lpy+3); cr.show_text("L")

            camx, camy = to_px(cx, cy)
            cr.set_source_rgb(0.2, 0.9, 0.3)
            cr.move_to(camx, camy-5)
            cr.line_to(camx-4, camy+4)
            cr.line_to(camx+4, camy+4)
            cr.close_path(); cr.fill()
            cr.set_font_size(9)
            cr.set_source_rgb(0.3, 1, 0.4)
            cr.move_to(camx+7, camy+3); cr.show_text("C")
        except Exception as e:
            logger.error("Robot diagram error: %s", e)
        return False

    # ---- Handlers ----

    def _on_robot_dims_changed(self, _spin):
        l_m = self._sp_robot_len.get_value() / 100.0
        w_m = self._sp_robot_wid.get_value() / 100.0
        h_m = self._sp_robot_hgt.get_value() / 100.0
        self._state.detection.set_robot_dims(l_m, w_m, h_m)
        self._state.send_ros_command({
            'type': 'set_robot_half_width', 'value': w_m / 2.0})
        if hasattr(self, '_ref_info_label'):
            self._update_ref_info()
        self._robot_da.queue_draw()
        self._schedule_save()

    def _on_sensor_pos_changed(self, _spin):
        self._state.detection.set_lidar_pos(
            self._sp_lidar_x.get_value() / 100.0,
            self._sp_lidar_y.get_value() / 100.0,
            self._sp_lidar_z.get_value() / 100.0)
        self._state.detection.set_camera_pos(
            self._sp_cam_x.get_value() / 100.0,
            self._sp_cam_y.get_value() / 100.0,
            self._sp_cam_z.get_value() / 100.0)
        if hasattr(self, '_ref_info_label'):
            self._update_ref_info()
        self._robot_da.queue_draw()
        self._schedule_save()

    def _on_ref_point_changed(self, combo):
        from target_nav.config import REF_POINTS
        idx = combo.get_active()
        if 0 <= idx < len(REF_POINTS):
            self._state.detection.set_ref_point(REF_POINTS[idx])
            self._update_ref_info()
            self._schedule_save()

    def _update_ref_info(self):
        try:
            cam_off = self._state.detection.get_sensor_offset('camera')
            lidar_off = self._state.detection.get_sensor_offset('lidar')
            self._ref_info_label.set_markup(
                f"<span size='small' foreground='#aaa'>"
                f"cam: {cam_off*100:+.1f}cm  "
                f"lidar: {lidar_off*100:+.1f}cm</span>")
        except Exception as e:
            logger.error("ref info error: %s", e)

    def _on_camera_yaw_changed(self, value):
        self._state.detection.set_camera_yaw(value)
        self._robot_da.queue_draw()
        self._schedule_save()

    def _on_lidar_yaw_changed(self, value):
        self._state.detection.set_lidar_yaw(value)
        self._robot_da.queue_draw()
        self._schedule_save()

    def _on_fov_changed(self, _spin):
        self._state.detection.set_camera_fov(self._sp_cam_fov.get_value())
        self._robot_da.queue_draw()
        self._schedule_save()

    def _on_target_changed(self, _spin):
        self._state.detection.set_target_dims(
            self._sp_tgt_h.get_value() / 100.0,
            self._sp_tgt_w.get_value() / 100.0)
        self._schedule_save()

    def _on_calibrate(self, _button):
        """Auto-calibrate distance estimator from the current best detection.

        Requires a live detection younger than 1 second. Uses the tallest
        bounding box as the reference measurement paired with the user-entered
        known distance from ``_cal_dist_spin``.
        """
        _, detections, _, det_age = self._state.detection.get_camera()
        if not detections or det_age > 1.0:
            self._cal_result.set_text("No detection!")
            return
        best = max(detections, key=lambda d: (d.bbox[3] - d.bbox[1]))
        box_h = best.bbox[3] - best.bbox[1]
        if box_h <= 0:
            self._cal_result.set_text("Invalid bbox")
            return
        dist = self._cal_dist_spin.get_value()
        self._state.detection.set_calibration(float(box_h), dist)
        self._cal_label.set_text(f"Current: {box_h:.0f}px at {dist:.2f}m")
        self._manual_h_spin.set_value(box_h)
        self._manual_d_spin.set_value(dist)
        self._cal_result.set_text(f"OK: {box_h:.0f}px @ {dist:.2f}m")
        self._schedule_save()

    def _on_manual_cal(self, _button):
        box_h = self._manual_h_spin.get_value()
        dist = self._manual_d_spin.get_value()
        self._state.detection.set_calibration(float(box_h), dist)
        self._cal_label.set_text(f"Current: {box_h:.0f}px at {dist:.2f}m")
        self._cal_result.set_text(f"Applied: {box_h:.0f}px @ {dist:.2f}m")
        self._schedule_save()

    def _on_reset_hardware_defaults(self):
        from target_nav.config import (
            DEFAULT_ROBOT, DEFAULT_LIDAR, DEFAULT_LIDAR_YAW,
            DEFAULT_CAMERA, DEFAULT_CAMERA_YAW, DEFAULT_CAMERA_FOV,
            DEFAULT_TARGET, REF_POINTS, DEFAULT_REF_POINT,
            DEFAULT_REF_BOX_HEIGHT, DEFAULT_REF_DISTANCE)

        self._sp_robot_len.set_value(DEFAULT_ROBOT['length'] * 100)
        self._sp_robot_wid.set_value(DEFAULT_ROBOT['width'] * 100)
        self._sp_robot_hgt.set_value(DEFAULT_ROBOT['height'] * 100)
        self._sp_lidar_x.set_value(DEFAULT_LIDAR['x'] * 100)
        self._sp_lidar_y.set_value(DEFAULT_LIDAR['y'] * 100)
        self._sp_lidar_z.set_value(DEFAULT_LIDAR['z'] * 100)
        self._state.detection.set_lidar_yaw(DEFAULT_LIDAR_YAW)
        self._set_widget_value('lidar_yaw', DEFAULT_LIDAR_YAW)
        self._sp_cam_x.set_value(DEFAULT_CAMERA['x'] * 100)
        self._sp_cam_y.set_value(DEFAULT_CAMERA['y'] * 100)
        self._sp_cam_z.set_value(DEFAULT_CAMERA['z'] * 100)
        self._state.detection.set_camera_yaw(DEFAULT_CAMERA_YAW)
        self._set_widget_value('camera_yaw', DEFAULT_CAMERA_YAW)
        self._sp_cam_fov.set_value(DEFAULT_CAMERA_FOV)
        self._sp_tgt_h.set_value(DEFAULT_TARGET['height'] * 100)
        self._sp_tgt_w.set_value(DEFAULT_TARGET['width'] * 100)
        self._state.detection.set_calibration(DEFAULT_REF_BOX_HEIGHT, DEFAULT_REF_DISTANCE)
        self._cal_label.set_text(
            f"Current: {DEFAULT_REF_BOX_HEIGHT:.0f}px at {DEFAULT_REF_DISTANCE:.2f}m")
        self._manual_h_spin.set_value(DEFAULT_REF_BOX_HEIGHT)
        self._manual_d_spin.set_value(DEFAULT_REF_DISTANCE)
        self._state.detection.set_ref_point(DEFAULT_REF_POINT)
        self._ref_combo.set_active(REF_POINTS.index(DEFAULT_REF_POINT))
        self._update_ref_info()
