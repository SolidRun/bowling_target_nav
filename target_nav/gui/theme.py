"""GTK3 dark theme CSS for V2N Robot Control GUI.

Provides a GitHub-dark-inspired color scheme applied globally via
CssProvider at ``STYLE_PROVIDER_PRIORITY_USER``. Covers both the main
window (buttons, labels, status bar) and the settings window (notebook
tabs, sliders, switches, spinbuttons, comboboxes, separators).

Uses ``background-image: none`` on all elements to override GTK3 Adwaita
theme's gradient, which otherwise hides our ``background-color`` on
Weston (the V2N board's Wayland compositor).

Runs in: GUI process (Process 0). ``apply_theme()`` must be called after
``Gtk.init()`` and before showing any windows.

Key function:
    apply_theme() -- load and register the dark CSS globally.

Module constant:
    DARK_THEME_CSS -- raw CSS bytes defining the complete dark theme.
"""

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk

DARK_THEME_CSS = b"""
window {
    background-color: #0d1117;
    background-image: none;
}
button {
    font-size: 16px;
    font-weight: bold;
    border-radius: 6px;
    padding: 6px 12px;
    border: 1px solid #30363d;
    background-color: #21262d;
    background-image: none;
    color: #c9d1d9;
}
button:hover {
    background-color: #30363d;
    background-image: none;
    border-color: #8b949e;
}
button.suggested-action {
    background-color: #238636;
    background-image: none;
    color: #ffffff;
    border-color: #238636;
}
button.suggested-action:hover {
    background-color: #2ea043;
    background-image: none;
}
button.destructive-action {
    background-color: #da3633;
    background-image: none;
    color: #ffffff;
    border-color: #da3633;
}
button.destructive-action:hover {
    background-color: #f85149;
    background-image: none;
}
button.settings-btn {
    background-color: #1f6feb;
    background-image: none;
    color: #ffffff;
    border-color: #1f6feb;
}
button.settings-btn:hover {
    background-color: #388bfd;
    background-image: none;
}
button.quit-btn {
    background-color: #6e4000;
    background-image: none;
    color: #ffa657;
    border-color: #6e4000;
}
button.quit-btn:hover {
    background-color: #845000;
    background-image: none;
}
label {
    color: #e6edf3;
    font-size: 14px;
}
.status-label {
    font-size: 15px;
    font-weight: bold;
    color: #58a6ff;
}

/* ---- Settings (embedded in main window) ---- */
notebook header {
    background-color: #0d1117;
    background-image: none;
    border-bottom: 1px solid #21262d;
}
notebook header tab {
    background-color: #0d1117;
    background-image: none;
    padding: 8px 12px;
    border: none;
    border-bottom: 3px solid transparent;
}
notebook header tab:checked {
    background-color: #161b22;
    background-image: none;
    border-bottom: 3px solid #1f6feb;
}
notebook header tab label {
    font-size: 14px;
    font-weight: bold;
    color: #8b949e;
}
notebook header tab:checked label {
    color: #58a6ff;
}

/* Scrolled content area */
notebook > stack {
    background-color: #161b22;
    background-image: none;
}
scrolledwindow {
    background-color: #161b22;
    background-image: none;
}
scrolledwindow viewport {
    background-color: #161b22;
    background-image: none;
}

/* Scale/slider */
scale trough {
    background-color: #21262d;
    background-image: none;
    border-radius: 4px;
    min-height: 10px;
}
scale highlight {
    background-color: #1f6feb;
    background-image: none;
    border-radius: 4px;
    min-height: 10px;
}
scale slider {
    background-color: #c9d1d9;
    background-image: none;
    border-radius: 50%;
    min-width: 22px;
    min-height: 22px;
}

/* Spin button */
spinbutton {
    background-color: #21262d;
    background-image: none;
    color: #e6edf3;
    border: 1px solid #30363d;
    border-radius: 4px;
    font-size: 14px;
    min-height: 30px;
}
spinbutton text {
    background-color: #21262d;
    background-image: none;
    color: #e6edf3;
}

/* Buttons inside settings */
button {
    background-color: #21262d;
    background-image: none;
    color: #c9d1d9;
    border: 1px solid #30363d;
    font-size: 14px;
    min-height: 34px;
}
button:hover {
    background-color: #30363d;
    background-image: none;
}

/* Switch */
switch {
    min-width: 48px;
    min-height: 24px;
}

/* Separator */
separator {
    background-color: #30363d;
}

/* ComboBox */
combobox button {
    min-height: 30px;
}
"""


def apply_theme():
    """Load and apply the dark theme CSS globally.

    Must be called after Gtk.init() so that Gdk.Screen.get_default()
    returns a valid screen. Registers the CSS at USER priority so it
    overrides the default Adwaita theme.
    """
    style_provider = Gtk.CssProvider()
    style_provider.load_from_data(DARK_THEME_CSS)
    Gtk.StyleContext.add_provider_for_screen(
        Gdk.Screen.get_default(),
        style_provider,
        Gtk.STYLE_PROVIDER_PRIORITY_USER
    )
