#!/usr/bin/env python3
"""
V2N Robot Control GUI - Entry Point
====================================

Multiprocessing architecture (3 processes, one per CPU core):
- Process 0 (main): GTK event loop + rendering
- Process 1: ROS2 node (map, scan, TF, navigation)
- Process 2: Camera capture + DRP-AI detection

Falls back to threading if BOWLING_NAV_MULTIPROCESS=0.

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

from bowling_target_nav.gui.theme import apply_theme
from bowling_target_nav.gui.main_window import MainGUI

# Feature flag: set to "0" to fall back to threading
_USE_MULTIPROCESS = os.environ.get('BOWLING_NAV_MULTIPROCESS', '1') != '0'

# Globals for cleanup
_processes = []
_threads = []
_hub = None
_gui_state = None


def _reset_cartographer_map():
    """Reset cartographer map by finishing old trajectory and starting new one.

    If cartographer is not running, launches it instead.
    Runs in a background thread to avoid blocking GUI startup.
    """
    import subprocess

    def _do_reset():
        result = subprocess.run(
            ["pgrep", "-f", "cartographer_node"],
            capture_output=True, text=True)

        if result.returncode == 0:
            print("[Main] Resetting cartographer trajectory...", flush=True)
            env = os.environ.copy()
            shell_prefix = (
                "source /opt/ros/humble/setup.bash && "
                "source ~/ros2_ws/install/setup.bash && ")
            subprocess.run(
                ["bash", "-c", shell_prefix +
                 "ros2 service call /finish_trajectory "
                 "cartographer_ros_msgs/srv/FinishTrajectory "
                 "'{trajectory_id: 0}'"],
                env=env, capture_output=True, timeout=5)
            subprocess.run(
                ["bash", "-c", shell_prefix +
                 "ros2 service call /start_trajectory "
                 "cartographer_ros_msgs/srv/StartTrajectory "
                 "\"{configuration_directory: "
                 "'/root/ros2_ws/install/bowling_target_nav/share/"
                 "bowling_target_nav/config', "
                 "configuration_basename: 'cartographer.lua', "
                 "use_initial_pose: false}\""],
                env=env, capture_output=True, timeout=5)
            print("[Main] Cartographer trajectory reset", flush=True)
        else:
            print("[Main] Launching cartographer...", flush=True)
            try:
                subprocess.Popen(
                    ["bash", "-c",
                     "source /opt/ros/humble/setup.bash && "
                     "source ~/ros2_ws/install/setup.bash && "
                     "ros2 launch bowling_target_nav mapping.launch.py"],
                    env=os.environ.copy(),
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    start_new_session=True)
                print("[Main] Cartographer launched", flush=True)
            except Exception as e:
                print(f"[Main] WARNING: Could not launch cartographer: {e}",
                      flush=True)

    threading.Thread(target=_do_reset, daemon=True).start()


# ==================================================================
# Multiprocess mode
# ==================================================================

def _cleanup_multiprocess():
    """Cleanup for multiprocess mode."""
    global _hub
    print("\n[Main] Cleanup started (multiprocess)...", flush=True)
    if _hub:
        _hub.shutdown.set()
    for p in _processes:
        if p.is_alive():
            p.join(timeout=5.0)
            if p.is_alive():
                print(f"[Main] Force-terminating {p.name}...", flush=True)
                p.terminate()
                p.join(timeout=2.0)
    if _hub:
        _hub.cleanup()
    print("[Main] Cleanup complete", flush=True)


def _signal_handler_mp(sig, frame):
    """Signal handler for multiprocess mode."""
    print(f"\n[Main] Signal {sig} received", flush=True)
    if _hub:
        _hub.shutdown.set()
    try:
        GLib.idle_add(Gtk.main_quit)
    except Exception:
        pass


def _watchdog_mp():
    """Monitor child processes — shut down if any crash unexpectedly."""
    while _hub and not _hub.shutdown.is_set():
        for p in _processes:
            if not p.is_alive() and not _hub.shutdown.is_set():
                print(f"[Main] Process {p.name} (pid={p.pid}) died unexpectedly!",
                      flush=True)
                _hub.shutdown.set()
                try:
                    GLib.idle_add(Gtk.main_quit)
                except Exception:
                    pass
                return
        time.sleep(1.0)


def main_multiprocess():
    """Launch GUI with ROS and Camera in separate processes."""
    global _hub, _gui_state, _processes
    import multiprocessing as mp

    # spawn avoids fork issues with GTK/rclpy
    mp.set_start_method('spawn', force=True)

    from bowling_target_nav.ipc.hub import IPCHub
    from bowling_target_nav.state.ipc_state import IPCSharedState
    from bowling_target_nav.processes.ros_process import ros_process_main
    from bowling_target_nav.processes.camera_process import camera_process_main

    _hub = IPCHub()
    _gui_state = IPCSharedState(_hub)

    atexit.register(_cleanup_multiprocess)
    signal.signal(signal.SIGINT, _signal_handler_mp)
    signal.signal(signal.SIGTERM, _signal_handler_mp)

    print("=" * 60, flush=True)
    print("V2N Robot Control - MULTIPROCESS MODE", flush=True)
    print("  Core 0: GUI (GTK rendering)", flush=True)
    print("  Core 1: ROS2 + Navigation", flush=True)
    print("  Core 2: Camera + DRP-AI Detection", flush=True)
    print("=" * 60, flush=True)
    print("Controls:", flush=True)
    print("  GO button / G key  : Navigate to detected target", flush=True)
    print("  STOP button / Space: Emergency stop", flush=True)
    print("  Q / ESC            : Quit", flush=True)
    print("=" * 60, flush=True)

    _reset_cartographer_map()

    # Pin GUI process to core 0
    try:
        os.sched_setaffinity(0, {0})
        print("[Main] GUI pinned to core 0", flush=True)
    except Exception:
        pass

    # Spawn child processes
    ros_p = mp.Process(
        target=ros_process_main, args=(_hub, 1),
        name="ROSProcess", daemon=False)
    ros_p.start()
    _processes.append(ros_p)
    print(f"[Main] ROS process started (pid={ros_p.pid})", flush=True)

    cam_p = mp.Process(
        target=camera_process_main, args=(_hub, 2),
        name="CameraProcess", daemon=False)
    cam_p.start()
    _processes.append(cam_p)
    print(f"[Main] Camera process started (pid={cam_p.pid})", flush=True)

    # Start watchdog thread
    threading.Thread(target=_watchdog_mp, name="Watchdog", daemon=True).start()

    if _hub.shutdown.is_set():
        return

    # Initialize GTK
    ok = Gtk.init_check(sys.argv)[0] if callable(getattr(Gtk, 'init_check', None)) else True
    if not ok:
        print(f"[Main] GTK init FAILED.", flush=True)
        print(f"[Main]   GDK_BACKEND={os.environ.get('GDK_BACKEND')}", flush=True)
        print(f"[Main]   WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY')}", flush=True)
        print(f"[Main]   XDG_RUNTIME_DIR={os.environ.get('XDG_RUNTIME_DIR')}", flush=True)
        print(f"[Main]   DISPLAY={os.environ.get('DISPLAY')}", flush=True)
        _hub.shutdown.set()
        return

    apply_theme()

    try:
        win = MainGUI(_gui_state)
        win.show_all()
        print("[Main] GUI running (multiprocess)", flush=True)
        Gtk.main()
    except Exception as e:
        print(f"[Main] GUI error: {e}", flush=True)
        print(f"[Main]   GDK_BACKEND={os.environ.get('GDK_BACKEND')}", flush=True)
        print(f"[Main]   WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY')}", flush=True)
        print(f"[Main]   XDG_RUNTIME_DIR={os.environ.get('XDG_RUNTIME_DIR')}", flush=True)
    finally:
        if _hub:
            _hub.shutdown.set()


# ==================================================================
# Threading mode (fallback)
# ==================================================================

def _cleanup_threading():
    """Cleanup for threading mode."""
    from bowling_target_nav.state import state
    print("\n[Main] Cleanup started (threading)...", flush=True)
    state.request_shutdown()
    for t in _threads:
        if t.is_alive():
            t.join(timeout=5.0)
    print("[Main] Cleanup complete", flush=True)


def _signal_handler_threading(sig, frame):
    from bowling_target_nav.state import state
    print(f"\n[Main] Signal {sig} received", flush=True)
    state.request_shutdown()
    try:
        GLib.idle_add(Gtk.main_quit)
    except Exception:
        pass


def main_threading():
    """Legacy threading mode — all on one core."""
    global _threads
    from bowling_target_nav.state import state
    from bowling_target_nav.threads.ros_node import ros_thread
    from bowling_target_nav.threads.camera_worker import camera_thread

    atexit.register(_cleanup_threading)
    signal.signal(signal.SIGINT, _signal_handler_threading)
    signal.signal(signal.SIGTERM, _signal_handler_threading)

    print("=" * 60, flush=True)
    print("V2N Robot Control - THREADING MODE (single core)", flush=True)
    print("  Set BOWLING_NAV_MULTIPROCESS=1 for multiprocess mode", flush=True)
    print("=" * 60, flush=True)
    print("Controls:", flush=True)
    print("  GO button / G key  : Navigate to detected target", flush=True)
    print("  STOP button / Space: Emergency stop", flush=True)
    print("  Q / ESC            : Quit", flush=True)
    print("=" * 60, flush=True)

    _reset_cartographer_map()

    ros_t = threading.Thread(target=ros_thread, name="ROS2Thread", daemon=False)
    ros_t.start()
    _threads.append(ros_t)

    cam_t = threading.Thread(target=lambda: camera_thread(state), name="CameraThread", daemon=False)
    cam_t.start()
    _threads.append(cam_t)

    if not state.running:
        return

    ok = Gtk.init_check(sys.argv)[0] if callable(getattr(Gtk, 'init_check', None)) else True
    if not ok:
        print(f"[Main] GTK init FAILED.", flush=True)
        state.request_shutdown()
        return

    apply_theme()

    try:
        win = MainGUI(state)
        win.show_all()
        print("[Main] GUI running (threading)", flush=True)
        Gtk.main()
    except Exception as e:
        print(f"[Main] GUI error: {e}", flush=True)
    finally:
        state.request_shutdown()


# ==================================================================
# Entry point
# ==================================================================

def main():
    if _USE_MULTIPROCESS:
        print("[Main] Multiprocess mode enabled", flush=True)
        main_multiprocess()
    else:
        print("[Main] Threading mode (BOWLING_NAV_MULTIPROCESS=0)", flush=True)
        main_threading()


if __name__ == '__main__':
    main()
