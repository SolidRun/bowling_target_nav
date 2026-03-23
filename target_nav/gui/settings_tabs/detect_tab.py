"""Sensors tab (Detection + Calibration + Mounting + DRP-AI) for SettingsWindow.

Provides the ``_build_sensors_page()`` method that creates a main tab
with four sub-tabs:

    Detection:   Python-side post-filter confidence, detection memory,
                 and legacy filter params (aspect, box size, bbox stability).
    Calibration: Distance calibration (auto + manual), target dimensions,
                 camera FOV, distance reference point.
    Mounting:    Robot body dimensions, LiDAR position + yaw, camera
                 position + yaw, robot diagram.
    DRP-AI:      C++ inference thresholds (confidence, NMS) and DRP-AI
                 hardware frequency.  Changes are written to config.ini
                 and the C++ subprocess is restarted.

The Calibration and Mounting sub-tabs are built by SetupTabMixin
(setup_tab.py). This mixin only builds the Detection and DRP-AI sub-tabs
and the overall Sensors page that combines all four.

Runs in: GUI process (Process 0) on the GTK main thread.
"""

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

from target_nav.config import (
    DEFAULT_DETECT_PARAMS, DEFAULT_DRPAI_PARAMS,
    DRPAI_DRP_FREQ_OPTIONS, DRPAI_AIMAC_FREQ_OPTIONS,
)
from target_nav.utils.log import get_logger

logger = get_logger('gui.settings.detect')


class DetectTabMixin:
    """Sensors tab mixin — builds Detection + DRP-AI sub-tabs and overall Sensors page."""

    def _build_sensors_page(self):
        """Build the Sensors tab with sub-tabs: Detection, Calibration, Mounting, DRP-AI."""
        detection_page = self._build_detection_subpage()
        calibration_page = self._build_calibration_subpage()
        mounting_page = self._build_mounting_subpage()
        drpai_page = self._build_drpai_subpage()

        return self._make_sub_notebook([
            ("Detection", detection_page),
            ("Calibration", calibration_page),
            ("Mounting", mounting_page),
            ("DRP-AI", drpai_page),
        ])

    # ================================================================
    # Detection sub-tab (Python-side post-filter)
    # ================================================================

    def _build_detection_subpage(self):
        """Build the Detection sub-tab with all filter parameters."""
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box.set_margin_top(4)

        det = self._state.detection

        # -- Core detection params --
        box.pack_start(self._make_section("AI Detection"), False, False, 0)

        box.pack_start(self._make_slider_row(
            'detect.confidence', "Confidence", 0.05, 0.95, 0.05,
            det.get_confidence_threshold(),
            self._on_detect_changed('set_confidence_threshold')),
            False, False, 0)

        box.pack_start(self._make_slider_row(
            'detect.expiry', "Detection Memory (s)", 0.3, 5.0, 0.1,
            det.get_detect_expiry(),
            self._on_detect_changed('set_detect_expiry')),
            False, False, 0)

        # Reset
        box.pack_start(self._make_reset_row(self._reset_detection_defaults),
                       False, False, 0)
        return box

    # ================================================================
    # DRP-AI sub-tab (C++ binary thresholds + hardware frequency)
    # ================================================================

    def _build_drpai_subpage(self):
        """Build the DRP-AI sub-tab with C++ inference params and restart button.

        Controls the DRP-AI C++ binary's config.ini parameters:
        - conf_threshold: minimum confidence for C++ inference (before Python sees results)
        - nms_threshold: NMS overlap threshold in C++ post-processing
        - drp_max_freq: DRP core clock frequency (affects power and inference speed)
        - drpai_freq: AI-MAC clock frequency (affects power and inference speed)

        Changes are staged in SettingsStore.  Pressing "Apply & Restart" writes
        the updated config.ini to disk and signals camera_worker to restart the
        C++ subprocess so the new values take effect.
        """
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        box.set_margin_top(4)

        det = self._state.detection
        drpai = det.get_drpai_params()

        # -- Thresholds --
        box.pack_start(self._make_section("C++ Inference Thresholds"), False, False, 0)

        box.pack_start(self._make_slider_row(
            'drpai.conf', "AI Confidence", 0.10, 0.95, 0.05,
            drpai.get('conf_threshold', DEFAULT_DRPAI_PARAMS['conf_threshold']),
            self._on_drpai_param_changed('conf_threshold')),
            False, False, 0)

        box.pack_start(self._make_slider_row(
            'drpai.nms', "NMS Threshold", 0.10, 0.95, 0.05,
            drpai.get('nms_threshold', DEFAULT_DRPAI_PARAMS['nms_threshold']),
            self._on_drpai_param_changed('nms_threshold')),
            False, False, 0)

        # -- Hardware frequency --
        box.pack_start(self._make_section("DRP-AI Hardware Frequency"), False, False, 0)

        # DRP Core frequency combo
        drp_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        drp_row.set_margin_start(10)
        drp_row.set_margin_end(10)
        drp_row.set_margin_top(2)
        drp_row.set_margin_bottom(2)
        drp_label = Gtk.Label()
        drp_label.set_markup("<span size='10000'>DRP Core</span>")
        drp_label.set_xalign(0)
        drp_label.set_size_request(140, -1)
        drp_row.pack_start(drp_label, False, False, 0)

        self._drp_freq_combo = Gtk.ComboBoxText()
        current_drp = drpai.get('drp_max_freq', DEFAULT_DRPAI_PARAMS['drp_max_freq'])
        for i, (val, label) in enumerate(sorted(DRPAI_DRP_FREQ_OPTIONS.items())):
            self._drp_freq_combo.append(str(val), f"{label} (level {val})")
            if val == current_drp:
                self._drp_freq_combo.set_active(i)
        self._drp_freq_combo.connect('changed', self._on_drp_freq_changed)
        drp_row.pack_start(self._drp_freq_combo, True, True, 0)
        box.pack_start(drp_row, False, False, 0)

        # AI-MAC frequency combo
        ai_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        ai_row.set_margin_start(10)
        ai_row.set_margin_end(10)
        ai_row.set_margin_top(2)
        ai_row.set_margin_bottom(2)
        ai_label = Gtk.Label()
        ai_label.set_markup("<span size='10000'>AI-MAC</span>")
        ai_label.set_xalign(0)
        ai_label.set_size_request(140, -1)
        ai_row.pack_start(ai_label, False, False, 0)

        self._aimac_freq_combo = Gtk.ComboBoxText()
        current_ai = drpai.get('drpai_freq', DEFAULT_DRPAI_PARAMS['drpai_freq'])
        for i, (val, label) in enumerate(sorted(DRPAI_AIMAC_FREQ_OPTIONS.items())):
            self._aimac_freq_combo.append(str(val), f"{label} (level {val})")
            if val == current_ai:
                self._aimac_freq_combo.set_active(i)
        self._aimac_freq_combo.connect('changed', self._on_aimac_freq_changed)
        ai_row.pack_start(self._aimac_freq_combo, True, True, 0)
        box.pack_start(ai_row, False, False, 0)

        # -- Info label --
        info_label = Gtk.Label()
        info_label.set_markup(
            "<span size='9000' foreground='#888'>"
            "Higher frequency = faster inference, more power.\n"
            "Changes require restarting the DRP-AI subprocess.</span>")
        info_label.set_xalign(0)
        info_label.set_margin_start(10)
        info_label.set_margin_top(6)
        box.pack_start(info_label, False, False, 0)

        # -- Apply & Restart button --
        btn_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        btn_row.set_margin_start(10)
        btn_row.set_margin_end(10)
        btn_row.set_margin_top(10)
        btn_row.set_margin_bottom(4)

        self._drpai_status_label = Gtk.Label(label="")
        self._drpai_status_label.set_xalign(0)
        btn_row.pack_start(self._drpai_status_label, True, True, 0)

        apply_btn = Gtk.Button(label="Apply & Restart DRP-AI")
        apply_btn.get_style_context().add_class('suggested-action')
        apply_btn.connect('clicked', self._on_apply_drpai)
        btn_row.pack_end(apply_btn, False, False, 0)

        reset_btn = Gtk.Button(label="Reset Defaults")
        reset_btn.connect('clicked', self._on_reset_drpai_defaults)
        btn_row.pack_end(reset_btn, False, False, 4)

        box.pack_start(btn_row, False, False, 0)

        return box

    # -- Detection callbacks --

    def _on_detect_changed(self, setter_name):
        """Return a callback that calls the named setter on detection store."""
        def cb(v):
            getattr(self._state.detection, setter_name)(v)
            logger.info("%s = %s", setter_name, v)
            self._schedule_save()
        return cb

    def _reset_detection_defaults(self):
        """Reset all detection params to defaults."""
        d = DEFAULT_DETECT_PARAMS
        pairs = [
            ('set_confidence_threshold', 'confidence_threshold', 'detect.confidence'),
            ('set_detect_expiry', 'detect_expiry', 'detect.expiry'),
        ]
        for setter, key, widget_key in pairs:
            getattr(self._state.detection, setter)(d[key])
            self._set_widget_value(widget_key, d[key])
        self._schedule_save()

    # -- DRP-AI callbacks --

    def _on_drpai_param_changed(self, param_key):
        """Return a callback that stages a DRP-AI param change in SettingsStore.

        The change is NOT applied until the user clicks "Apply & Restart".
        """
        def cb(v):
            self._state.detection.set_drpai_param(param_key, v)
            logger.info("DRP-AI staged: %s = %s", param_key, v)
        return cb

    def _on_drp_freq_changed(self, combo):
        """Handle DRP core frequency combo selection."""
        active_id = combo.get_active_id()
        if active_id is not None:
            self._state.detection.set_drpai_param('drp_max_freq', int(active_id))
            logger.info("DRP-AI staged: drp_max_freq = %s", active_id)

    def _on_aimac_freq_changed(self, combo):
        """Handle AI-MAC frequency combo selection."""
        active_id = combo.get_active_id()
        if active_id is not None:
            self._state.detection.set_drpai_param('drpai_freq', int(active_id))
            logger.info("DRP-AI staged: drpai_freq = %s", active_id)

    def _on_apply_drpai(self, btn):
        """Write updated DRP-AI config.ini and request subprocess restart.

        Steps:
        1. Write staged params to config.ini (atomic write).
        2. Persist to calibration.json so params survive app restart.
        3. Signal camera_worker to restart the C++ subprocess.
        4. In multiprocess mode, also publish /drpai_restart ROS2 topic.
        """
        det = self._state.detection

        # Write config.ini with updated params
        if det.write_drpai_config_ini():
            self._drpai_status_label.set_markup(
                "<span foreground='#3fb950'>Restarting DRP-AI...</span>")
            logger.info("DRP-AI config.ini updated, requesting restart")
        else:
            self._drpai_status_label.set_markup(
                "<span foreground='#f85149'>Failed to write config.ini</span>")
            logger.error("Failed to write DRP-AI config.ini")
            return

        # Persist to calibration.json
        self._schedule_save()

        # Signal camera_worker to restart the subprocess
        det.request_drpai_restart()

        # In multiprocess mode, also publish via ROS2 topic
        if hasattr(self._state, '_publish_drpai_restart'):
            self._state._publish_drpai_restart()

    def _on_reset_drpai_defaults(self, btn):
        """Reset DRP-AI params to defaults and update GUI widgets."""
        d = DEFAULT_DRPAI_PARAMS
        det = self._state.detection
        for key, val in d.items():
            det.set_drpai_param(key, val)

        # Update sliders
        self._set_widget_value('drpai.conf', d['conf_threshold'])
        self._set_widget_value('drpai.nms', d['nms_threshold'])

        # Update combos
        for i, val in enumerate(sorted(DRPAI_DRP_FREQ_OPTIONS.keys())):
            if val == d['drp_max_freq']:
                self._drp_freq_combo.set_active(i)
                break
        for i, val in enumerate(sorted(DRPAI_AIMAC_FREQ_OPTIONS.keys())):
            if val == d['drpai_freq']:
                self._aimac_freq_combo.set_active(i)
                break

        self._drpai_status_label.set_markup(
            "<span foreground='#888'>Reset to defaults (not applied yet)</span>")
        logger.info("DRP-AI params reset to defaults")
