"""Display backend setup -- auto-detect Wayland/X11 for V2N embedded board.

The V2N RZ/V2N board runs Weston (Wayland compositor) by default. On
development machines, X11 is typically used instead. This module detects
which compositor is running and sets the appropriate GDK_BACKEND and
WAYLAND_DISPLAY / XDG_RUNTIME_DIR environment variables so GTK3
connects to the correct display server.

MUST be called before importing ``gi.repository.Gtk`` (i.e., before
``Gtk.init()``). Called at module level in ``app/main.py``.

Also disables OpenCV OpenCL (``OPENCV_OPENCL_DEVICE=disabled``) to
prevent GPU contention with DRP-AI on the V2N board.

Runs in: GUI process (Process 0) at import time.

Key function:
    setup_display() -- detect display server and set environment variables.
"""

import os
import subprocess

from target_nav.utils.log import get_logger

logger = get_logger('gui.display')


def setup_display():
    """Auto-detect and configure display for V2N (Weston/Wayland/framebuffer).

    Detection priority:
      1. Weston running -> set GDK_BACKEND=wayland, locate socket
      2. DISPLAY env set -> set GDK_BACKEND=x11
      3. Neither -> try Wayland defaults as last resort
    """
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

        # Search common socket locations for the Wayland compositor.
        # The V2N board's Weston often runs as a system service (uid 996)
        # with the socket in /run/user/996 rather than the login user's dir.
        search_dirs = [
            os.environ.get('XDG_RUNTIME_DIR', ''),
            '/run',
            f'/run/user/{os.getuid()}',
            '/run/user/996',  # V2N system Weston service
            '/tmp',
        ]
        for d in search_dirs:
            if d and os.path.exists(os.path.join(d, wayland_display)):
                os.environ['XDG_RUNTIME_DIR'] = d
                socket_found = True
                logger.info("Weston socket found: %s/%s", d, wayland_display)
                break

        if not socket_found:
            os.environ.setdefault('XDG_RUNTIME_DIR', '/run')
            logger.warning("Wayland socket not found")

        logger.info("Weston detected -> Wayland (XDG_RUNTIME_DIR=%s)", os.environ.get('XDG_RUNTIME_DIR'))
    elif os.environ.get('DISPLAY'):
        os.environ.setdefault('GDK_BACKEND', 'x11')
        logger.info("X11 display: %s", os.environ['DISPLAY'])
    else:
        os.environ.setdefault('GDK_BACKEND', 'wayland')
        os.environ.setdefault('WAYLAND_DISPLAY', 'wayland-0')
        os.environ.setdefault('XDG_RUNTIME_DIR', '/run')
        logger.info("No display server found -> trying Wayland defaults")

    os.environ['OPENCV_OPENCL_DEVICE'] = 'disabled'
