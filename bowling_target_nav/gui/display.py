"""Display setup - auto-detect Wayland/X11 for V2N embedded board.

MUST be called before importing gi.repository.Gtk.
"""

import os
import subprocess


def setup_display():
    """Auto-detect and configure display for V2N (Weston/Wayland/framebuffer)."""
    weston_running = False
    try:
        result = subprocess.run(['pgrep', '-x', 'weston'], capture_output=True, timeout=2)
        weston_running = (result.returncode == 0)
    except Exception:
        pass

    if weston_running:
        os.environ['GDK_BACKEND'] = 'wayland'
        os.environ.setdefault('WAYLAND_DISPLAY', 'wayland-0')

        wayland_display = os.environ.get('WAYLAND_DISPLAY', 'wayland-0')
        socket_found = False

        search_dirs = [
            os.environ.get('XDG_RUNTIME_DIR', ''),
            '/run',
            f'/run/user/{os.getuid()}',
            '/run/user/996',
            '/tmp',
        ]
        for d in search_dirs:
            if d and os.path.exists(os.path.join(d, wayland_display)):
                os.environ['XDG_RUNTIME_DIR'] = d
                socket_found = True
                print(f"[Display] Weston socket found: {d}/{wayland_display}", flush=True)
                break

        if not socket_found:
            os.environ.setdefault('XDG_RUNTIME_DIR', '/run')
            print("[Display] WARNING: Wayland socket not found", flush=True)

        print(f"[Display] Weston detected -> Wayland (XDG_RUNTIME_DIR={os.environ.get('XDG_RUNTIME_DIR')})", flush=True)
    elif os.environ.get('DISPLAY'):
        os.environ.setdefault('GDK_BACKEND', 'x11')
        print(f"[Display] X11 display: {os.environ['DISPLAY']}", flush=True)
    else:
        os.environ.setdefault('GDK_BACKEND', 'wayland')
        os.environ.setdefault('WAYLAND_DISPLAY', 'wayland-0')
        os.environ.setdefault('XDG_RUNTIME_DIR', '/run')
        print("[Display] No display server found -> trying Wayland defaults", flush=True)

    os.environ['OPENCV_OPENCL_DEVICE'] = 'disabled'
