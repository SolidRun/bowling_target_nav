#!/usr/bin/env python3
"""
V2N Robot Control GUI - Entry Point
====================================

Starts three threads and runs the GTK main loop:
- Main thread: GTK event loop + rendering
- Thread 1: ROS2 node (map, scan, TF, navigation)
- Thread 2: Camera capture + YOLO detection

Usage:
    ros2 run bowling_target_nav main_gui
"""

import os
import sys
import signal
import threading
import time
import atexit

# Display setup MUST happen before GTK import
from bowling_target_nav.gui.display import setup_display
setup_display()

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib

from bowling_target_nav.state import state
from bowling_target_nav.threads.ros_node import ros_thread
from bowling_target_nav.threads.camera_worker import camera_thread
from bowling_target_nav.gui.theme import apply_theme
from bowling_target_nav.gui.main_window import MainGUI

# Non-daemon threads so finally blocks run on cleanup
_threads = []


def cleanup():
    print("\n[Main] Cleanup started...", flush=True)
    state.request_shutdown()
    for t in _threads:
        if t.is_alive():
            t.join(timeout=5.0)
    print("[Main] Cleanup complete", flush=True)


def signal_handler(sig, frame):
    """Signal handler: set shutdown flag, schedule GTK quit from main loop.

    Gtk.main_quit() is NOT reentrant and must not be called from signal context.
    GLib.idle_add() safely schedules it on the GTK main loop iteration.
    """
    print(f"\n[Main] Signal {sig} received", flush=True)
    state.request_shutdown()
    try:
        GLib.idle_add(Gtk.main_quit)
    except Exception:
        pass


def _wait_for_ros(timeout=5.0):
    """Wait for ROS node to initialize (up to timeout seconds)."""
    deadline = time.time() + timeout
    while time.time() < deadline and state.running:
        if state._ros_node is not None:
            print("[Main] ROS2 node ready", flush=True)
            return True
        time.sleep(0.1)
    if state._ros_node is None:
        print("[Main] WARNING: ROS2 node not ready after timeout", flush=True)
    return state._ros_node is not None


def main():
    global _threads

    atexit.register(cleanup)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 60, flush=True)
    print("V2N Robot Control - SLAM + Camera + Navigation", flush=True)
    print("=" * 60, flush=True)
    print("Controls:", flush=True)
    print("  GO button / G key  : Navigate to detected target", flush=True)
    print("  STOP button / Space: Emergency stop", flush=True)
    print("  Q / ESC            : Quit", flush=True)
    print("=" * 60, flush=True)

    # Start worker threads (daemon=False for clean shutdown)
    ros_t = threading.Thread(target=ros_thread, name="ROS2Thread", daemon=False)
    ros_t.start()
    _threads.append(ros_t)

    cam_t = threading.Thread(target=lambda: camera_thread(state), name="CameraThread", daemon=False)
    cam_t.start()
    _threads.append(cam_t)

    # Wait for ROS node to be ready (replaces blind sleep)
    _wait_for_ros(timeout=5.0)

    if not state.running:
        return

    # Initialize GTK
    ok = Gtk.init_check(sys.argv)[0] if callable(getattr(Gtk, 'init_check', None)) else True
    if not ok:
        print(f"[Main] GTK init FAILED.", flush=True)
        print(f"[Main]   GDK_BACKEND={os.environ.get('GDK_BACKEND')}", flush=True)
        print(f"[Main]   WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY')}", flush=True)
        print(f"[Main]   XDG_RUNTIME_DIR={os.environ.get('XDG_RUNTIME_DIR')}", flush=True)
        print(f"[Main]   DISPLAY={os.environ.get('DISPLAY')}", flush=True)
        state.request_shutdown()
        return

    apply_theme()

    try:
        win = MainGUI(state)
        win.show_all()
        print("[Main] GUI running", flush=True)
        Gtk.main()
    except Exception as e:
        print(f"[Main] GUI error: {e}", flush=True)
        print(f"[Main]   GDK_BACKEND={os.environ.get('GDK_BACKEND')}", flush=True)
        print(f"[Main]   WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY')}", flush=True)
        print(f"[Main]   XDG_RUNTIME_DIR={os.environ.get('XDG_RUNTIME_DIR')}", flush=True)
    finally:
        state.request_shutdown()


if __name__ == '__main__':
    main()
