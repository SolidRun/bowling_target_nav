"""Settings tab mixins for SettingsWindow.

Each mixin class provides ``_build_*_page()`` methods that construct
Notebook tabs (with optional sub-tabs inside), plus callbacks for
that tab's widgets. The mixins are combined via multiple inheritance
in ``SettingsWindow``.

Tabs (5 main, with sub-tabs):
    Navigate  -- Sub-tabs: Speed, Target, Approach.
    Search    -- Sub-tabs: Scan, Spiral.
    Sensors   -- Sub-tabs: Detection, Calibration, Mounting.
    Map       -- Flat page (rotation, overlays, sizes).
    Tools     -- Sub-tabs: Drive, Motors, System.
"""

from target_nav.gui.settings_tabs.nav_tab import NavTabMixin
from target_nav.gui.settings_tabs.detect_tab import DetectTabMixin
from target_nav.gui.settings_tabs.map_tab import MapTabMixin
from target_nav.gui.settings_tabs.setup_tab import SetupTabMixin
from target_nav.gui.settings_tabs.tools_tab import ToolsTabMixin

__all__ = [
    'NavTabMixin',
    'DetectTabMixin',
    'MapTabMixin',
    'SetupTabMixin',
    'ToolsTabMixin',
]
