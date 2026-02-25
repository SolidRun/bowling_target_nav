"""GTK3 dark theme CSS for V2N Robot Control GUI.

Uses background-image: none on all elements to override GTK3 Adwaita
theme's gradient, which otherwise hides our background-color on Weston.
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
.settings-window {
    background-color: #161b22;
    background-image: none;
}
.settings-window label {
    color: #e6edf3;
    font-size: 13px;
}
.settings-window notebook {
    background-color: #161b22;
    background-image: none;
}
.settings-window notebook header {
    background-color: #0d1117;
    background-image: none;
}
.settings-window notebook tab {
    background-color: #21262d;
    background-image: none;
    color: #c9d1d9;
    padding: 6px 14px;
    border: 1px solid #30363d;
}
.settings-window notebook tab:checked {
    background-color: #161b22;
    background-image: none;
    color: #58a6ff;
    border-bottom-color: #58a6ff;
}
.settings-window notebook stack {
    background-color: #161b22;
    background-image: none;
}
.settings-window scale trough {
    background-color: #21262d;
    background-image: none;
    border-radius: 4px;
}
.settings-window scale highlight {
    background-color: #1f6feb;
    background-image: none;
    border-radius: 4px;
}
.settings-window scale slider {
    background-color: #c9d1d9;
    background-image: none;
    border-radius: 50%;
}
.settings-window spinbutton {
    background-color: #21262d;
    background-image: none;
    color: #e6edf3;
    border: 1px solid #30363d;
    border-radius: 4px;
}
.settings-window spinbutton text {
    background-color: #21262d;
    background-image: none;
    color: #e6edf3;
}
.settings-window button {
    background-color: #21262d;
    background-image: none;
    color: #c9d1d9;
    border: 1px solid #30363d;
}
.settings-window button:hover {
    background-color: #30363d;
    background-image: none;
}
"""


def apply_theme():
    """Apply the dark theme CSS globally. Call after Gtk.init()."""
    style_provider = Gtk.CssProvider()
    style_provider.load_from_data(DARK_THEME_CSS)
    Gtk.StyleContext.add_provider_for_screen(
        Gdk.Screen.get_default(),
        style_provider,
        Gtk.STYLE_PROVIDER_PRIORITY_USER
    )
