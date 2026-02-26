"""Settings popup window with parameter sliders and calibration tools."""

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


class SettingsWindow(Gtk.Window):
    """Settings popup with parameter sliders and calibration tools.

    Five tabs: Navigation, Blind Approach, Detection, Obstacle, Calibration.
    Navigation/Blind/Obstacle sliders write to state._ros_node.navigator attrs.
    Detection sliders write to state.detection store methods.
    """

    def __init__(self, shared_state):
        super().__init__(title="Robot Settings")
        self._state = shared_state

        self.set_default_size(540, 620)
        self.set_position(Gtk.WindowPosition.CENTER)
        self.set_type_hint(Gdk.WindowTypeHint.DIALOG)
        self.get_style_context().add_class('settings-window')

        self.connect('delete-event', self._on_delete)
        self._motor_test_timer = None

        notebook = Gtk.Notebook()
        self.add(notebook)

        notebook.append_page(self._build_nav_tab(), Gtk.Label(label="Navigation"))
        notebook.append_page(self._build_blind_tab(), Gtk.Label(label="Blind Approach"))
        notebook.append_page(self._build_detection_tab(), Gtk.Label(label="Detection"))
        notebook.append_page(self._build_obstacle_tab(), Gtk.Label(label="Obstacle"))
        notebook.append_page(self._build_calibration_tab(), Gtk.Label(label="Calibration"))

    def _on_delete(self, widget, event):
        # Cancel any pending motor test timer to prevent orphaned GLib source
        if self._motor_test_timer:
            GLib.source_remove(self._motor_test_timer)
            self._motor_test_timer = None
            # Also stop motors when closing settings mid-test
            node = self._state._ros_node
            if node:
                node.cmd_vel_pub.publish(Twist())
        self.hide()
        return True

    # ---- Slider helper ----

    def _make_slider(self, label_text, min_val, max_val, step, default, callback):
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        row.set_margin_start(10)
        row.set_margin_end(10)
        row.set_margin_top(4)
        row.set_margin_bottom(4)

        label = Gtk.Label(label=label_text)
        label.set_xalign(0)
        label.set_size_request(200, -1)
        row.pack_start(label, False, False, 0)

        adj = Gtk.Adjustment(value=default, lower=min_val, upper=max_val,
                             step_increment=step, page_increment=step * 5)
        scale = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL, adjustment=adj)
        scale.set_digits(2 if step < 0.1 else 1)
        scale.set_value_pos(Gtk.PositionType.RIGHT)
        row.pack_start(scale, True, True, 0)

        val_label = Gtk.Label(label=f"{default:.2f}")
        val_label.set_size_request(60, -1)
        row.pack_start(val_label, False, False, 0)

        def on_change(s):
            v = s.get_value()
            val_label.set_text(f"{v:.2f}")
            callback(v)

        scale.connect('value-changed', on_change)
        return row

    def _make_section_label(self, text):
        label = Gtk.Label()
        label.set_markup(
            f"<span foreground='#58a6ff' font_weight='bold' size='large'>{text}</span>")
        label.set_xalign(0)
        label.set_margin_start(10)
        label.set_margin_top(14)
        label.set_margin_bottom(6)
        return label

    # ---- Navigator attribute setter ----

    def _nav_setter(self, attr):
        """Return a callback that sets navigator.attr when the slider changes."""
        def cb(v):
            node = self._state._ros_node
            if node and hasattr(node, 'navigator'):
                setattr(node.navigator, attr, v)
        return cb

    # ---- Tab builders ----

    def _build_nav_tab(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        s = self._nav_setter
        box.pack_start(self._make_slider("Linear Speed (m/s)", 0.05, 0.30, 0.01, 0.15, s('linear_speed')), False, False, 0)
        box.pack_start(self._make_slider("Min Speed (m/s)", 0.05, 0.20, 0.01, 0.10, s('min_linear_speed')), False, False, 0)
        box.pack_start(self._make_slider("Angular Speed (rad/s)", 0.1, 1.0, 0.05, 0.5, s('angular_speed')), False, False, 0)
        box.pack_start(self._make_slider("Approach Distance (m)", 0.05, 0.50, 0.01, 0.15, s('approach_distance')), False, False, 0)
        box.pack_start(self._make_slider("Lost Timeout (s)", 1.0, 10.0, 0.5, 3.0, s('lost_timeout')), False, False, 0)
        box.pack_start(self._make_slider("Search Timeout (s)", 10.0, 60.0, 5.0, 30.0, s('search_timeout')), False, False, 0)
        box.pack_start(self._make_slider("Search Angular Speed", 0.1, 1.0, 0.05, 0.4, s('search_angular_speed')), False, False, 0)
        return box

    def _build_blind_tab(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        s = self._nav_setter
        box.pack_start(self._make_slider("Entry Distance (m)", 0.30, 1.50, 0.05, 0.80, s('blind_approach_entry_distance')), False, False, 0)
        box.pack_start(self._make_slider("Approach Speed (m/s)", 0.05, 0.20, 0.01, 0.10, s('blind_approach_speed')), False, False, 0)
        box.pack_start(self._make_slider("Timeout (s)", 3.0, 15.0, 0.5, 8.0, s('blind_approach_timeout')), False, False, 0)
        box.pack_start(self._make_slider("LiDAR Stop (m)", 0.05, 0.30, 0.01, 0.12, s('blind_approach_lidar_stop')), False, False, 0)
        box.pack_start(self._make_slider("Arrival Margin (m)", 0.05, 0.25, 0.01, 0.10, s('blind_approach_arrival_margin')), False, False, 0)
        return box

    def _build_detection_tab(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        det = self._state.detection
        box.pack_start(self._make_slider("Confidence Threshold", 0.10, 0.90, 0.05, 0.35,
                                         lambda v: det.set_confidence_threshold(v)), False, False, 0)
        box.pack_start(self._make_slider("Detection Interval (s)", 0.5, 5.0, 0.1, 2.0,
                                         lambda v: det.set_detect_interval(v)), False, False, 0)
        box.pack_start(self._make_slider("Detection Expiry (s)", 0.5, 5.0, 0.1, 1.5,
                                         lambda v: det.set_detect_expiry(v)), False, False, 0)
        return box

    def _build_obstacle_tab(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        s = self._nav_setter
        box.pack_start(self._make_slider("Stop Distance (m)", 0.10, 0.50, 0.01, 0.25, s('obstacle_distance')), False, False, 0)
        box.pack_start(self._make_slider("Slowdown Distance (m)", 0.20, 1.00, 0.05, 0.50, s('obstacle_slowdown_distance')), False, False, 0)
        box.pack_start(self._make_slider("Robot Half Width (m)", 0.05, 0.30, 0.01, 0.18, s('robot_half_width')), False, False, 0)
        return box

    def _build_calibration_tab(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)

        # Distance Calibration
        box.pack_start(self._make_section_label("Distance Calibration"), False, False, 0)

        info = Gtk.Label(label="Place pin at known distance, then press Calibrate")
        info.set_xalign(0)
        info.set_margin_start(10)
        box.pack_start(info, False, False, 4)

        cal_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        cal_row.set_margin_start(10)
        cal_row.set_margin_end(10)

        cal_row.pack_start(Gtk.Label(label="Known Distance (m):"), False, False, 0)
        self.cal_distance_spin = Gtk.SpinButton.new_with_range(0.3, 3.0, 0.1)
        self.cal_distance_spin.set_value(1.0)
        cal_row.pack_start(self.cal_distance_spin, False, False, 0)

        cal_btn = Gtk.Button(label="CALIBRATE")
        cal_btn.connect('clicked', self._on_calibrate)
        cal_row.pack_start(cal_btn, False, False, 10)

        self.cal_result_label = Gtk.Label(label="")
        cal_row.pack_start(self.cal_result_label, True, True, 0)
        box.pack_start(cal_row, False, False, 4)

        # Motor Test
        box.pack_start(self._make_section_label("Motor Test"), False, False, 0)

        speed_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        speed_row.set_margin_start(10)
        speed_row.set_margin_end(10)
        speed_row.pack_start(Gtk.Label(label="Test Speed (m/s):"), False, False, 0)
        self.test_speed_spin = Gtk.SpinButton.new_with_range(0.05, 0.30, 0.01)
        self.test_speed_spin.set_value(0.10)
        self.test_speed_spin.set_digits(2)
        speed_row.pack_start(self.test_speed_spin, False, False, 0)
        box.pack_start(speed_row, False, False, 4)

        btn_grid = Gtk.Grid()
        btn_grid.set_column_spacing(8)
        btn_grid.set_row_spacing(8)
        btn_grid.set_margin_start(10)
        btn_grid.set_margin_end(10)

        motor_tests = [
            ("FORWARD", 0, 0), ("BACKWARD", 0, 1),
            ("LEFT", 1, 0), ("RIGHT", 1, 1),
            ("TURN L", 2, 0), ("TURN R", 2, 1),
        ]
        for label, row, col in motor_tests:
            btn = Gtk.Button(label=label)
            btn.set_size_request(120, 40)
            btn.connect('clicked', self._on_motor_test, label)
            btn_grid.attach(btn, col, row, 1, 1)

        box.pack_start(btn_grid, False, False, 4)

        self.motor_status_label = Gtk.Label(label="")
        self.motor_status_label.set_xalign(0)
        self.motor_status_label.set_margin_start(10)
        box.pack_start(self.motor_status_label, False, False, 4)

        # Odometry Reset
        box.pack_start(self._make_section_label("Odometry"), False, False, 0)

        odom_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        odom_row.set_margin_start(10)
        reset_btn = Gtk.Button(label="RESET ODOMETRY")
        reset_btn.set_size_request(180, 40)
        reset_btn.connect('clicked', self._on_reset_odom)
        odom_row.pack_start(reset_btn, False, False, 0)
        self.odom_status_label = Gtk.Label(label="")
        odom_row.pack_start(self.odom_status_label, True, True, 0)
        box.pack_start(odom_row, False, False, 4)

        return box

    # ---- Handlers ----

    def _on_calibrate(self, button):
        _, detections, _, det_age = self._state.detection.get_camera()
        if not detections or det_age > 1.0:
            self.cal_result_label.set_text("No detection visible!")
            return

        best = max(detections, key=lambda d: (d['bbox'][3] - d['bbox'][1]))
        x1, y1, x2, y2 = best['bbox']
        box_height = y2 - y1

        if box_height <= 0:
            self.cal_result_label.set_text("Invalid detection bbox")
            return

        known_dist = self.cal_distance_spin.get_value()
        self._state.detection.set_calibration(float(box_height), known_dist)
        self.cal_result_label.set_text(f"Calibrated: {box_height}px at {known_dist:.1f}m")

    def _on_motor_test(self, button, direction):
        node = self._state._ros_node
        if not node:
            self.motor_status_label.set_text("ROS node not ready")
            return

        speed = self.test_speed_spin.get_value()
        cmd = Twist()

        if direction == "FORWARD":
            cmd.linear.x = speed
        elif direction == "BACKWARD":
            cmd.linear.x = -speed
        elif direction == "LEFT":
            cmd.linear.y = speed
        elif direction == "RIGHT":
            cmd.linear.y = -speed
        elif direction == "TURN L":
            cmd.angular.z = 0.5
        elif direction == "TURN R":
            cmd.angular.z = -0.5

        node.cmd_vel_pub.publish(cmd)
        self.motor_status_label.set_text(f"Testing {direction}...")

        if self._motor_test_timer:
            GLib.source_remove(self._motor_test_timer)

        def stop_motor():
            node.cmd_vel_pub.publish(Twist())
            self.motor_status_label.set_text(f"{direction} done")
            self._motor_test_timer = None
            return False

        self._motor_test_timer = GLib.timeout_add(1000, stop_motor)

    def _on_reset_odom(self, button):
        node = self._state._ros_node
        if not node:
            self.odom_status_label.set_text("ROS node not ready")
            return

        if not hasattr(node, '_reset_odom_pub'):
            node._reset_odom_pub = node.create_publisher(Empty, '/reset_odom', 1)

        node._reset_odom_pub.publish(Empty())
        self.odom_status_label.set_text("Odometry reset sent")
