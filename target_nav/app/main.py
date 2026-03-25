#!/usr/bin/env python3
"""V2N Robot Control GUI -- application entry point.

This module is the top-level entry point for the target navigation
system.  Starts the GTK3 GUI and spawns 2 worker processes for ROS2
navigation and camera/detection.

Architecture (3-process mode):
    Process 0 (Core 0): Pure GTK -- event loop, Cairo rendering, user
        interaction.  No ROS2 in this process.  A lightweight ShmPoll
        daemon thread reads struct-based shared memory directly from
        Nav and Camera processes and populates a local SharedState for
        the GTK panels to read.  Commands (GO, STOP) are written to a
        lock-free ring buffer in SHM for the Nav process to consume.
    Process 1 (Core 1): NavNode -- rclpy spin, TF2, navigation
        (VFH, Kalman, blind approach, search), Arduino driver.
        Writes nav state and LiDAR points to SHM via NavShmWriter
        and LaserShmWriter.  Reads commands from CmdRingBuffer.
    Process 2 (Core 2): CameraNode -- DRP-AI inference, detection
        tracking, distance estimation.  Writes detections to SHM
        via DetShmWriter.

    Each process creates its own rclpy context.  Inter-process
    communication uses struct-based SHM with torn-read/write
    protection via sequence numbers.  No Bridge process is needed.

Key functions:
    main()                -- entry point.
    main_multiprocess()   -- 3-process architecture.

Related modules:
    app/nav_node.py       -- spawned as Process 1 (navigation + TF).
    app/camera_node.py    -- spawned as Process 2 (camera + DRP-AI).
    gui/main_window.py    -- MainGUI GTK3 window on Process 0.
    ipc/shm_struct.py     -- struct-based SHM readers/writers.
    state/shared_state.py -- SharedState populated by ShmPoll thread.
"""

import json
import os
import sys
import signal
import threading
import time
import atexit

from target_nav.utils.log import get_logger

logger = get_logger('app.main')

# Display setup MUST happen before GTK import
from target_nav.gui.display import setup_display
setup_display()

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib

from target_nav.gui.theme import apply_theme
from target_nav.gui.main_window import MainGUI

# Globals for cleanup
_processes = []
_threads = []
_shutdown_event = None
_gui_state = None


# ==================================================================
# Multiprocess mode
# ==================================================================

def _cleanup_multiprocess():
    """Clean shutdown: stop motors, kill DRP-AI, terminate all child processes."""
    global _shutdown_event
    logger.info("Cleanup started (multiprocess)...")

    # 1. Signal everything to stop
    if _shutdown_event:
        _shutdown_event.set()
    if _gui_state:
        _gui_state.close()

    # 2. Kill DRP-AI subprocess (child of camera_node, not tracked in _processes)
    try:
        import subprocess
        subprocess.run(['pkill', '-9', '-f', 'app_yolo_cam'],
                       timeout=2, capture_output=True)
    except Exception:
        pass

    # 3. Terminate child processes (nav_node, camera_node)
    for p in _processes:
        if p.is_alive():
            p.terminate()  # SIGTERM first
    # Give them 2s to exit gracefully
    for p in _processes:
        p.join(timeout=2.0)
    # Force-kill any survivors
    for p in _processes:
        if p.is_alive():
            logger.info("Force-killing %s", p.name)
            p.kill()  # SIGKILL
            p.join(timeout=1.0)

    logger.info("Cleanup complete — all processes terminated")


def _signal_handler_mp(sig, frame):
    """Signal handler for multiprocess mode -- set shutdown flag and quit GTK.

    Idempotent: repeated signals (e.g., double Ctrl+C) are safe.
    """
    if _shutdown_event and _shutdown_event.is_set():
        return  # Already shutting down — ignore repeated signals
    logger.info("Signal %s received", sig)
    if _shutdown_event:
        _shutdown_event.set()
    if _gui_state:
        _gui_state.close()
    try:
        GLib.idle_add(Gtk.main_quit)
    except Exception:
        pass


def _watchdog_mp(shutdown_event):
    """Monitor child processes -- handle crashes with graceful degradation.

    Runs in a daemon thread, polling process liveness every 1 second.

    Graceful camera degradation: If the Camera process (DRP-AI + capture)
    crashes, the robot continues navigating using the last known target
    position and search/spiral behaviors. Only new detections are lost.

    Safety-critical Nav process: If the Nav process (TF2, navigator,
    Arduino driver) dies, motors are no longer being commanded. The
    watchdog triggers a full shutdown via shutdown_event + Gtk.main_quit
    to prevent uncontrolled robot state.
    """
    while not shutdown_event.is_set():
        for p in _processes:
            if not p.is_alive() and not shutdown_event.is_set():
                if p.name == "CameraProcess":
                    logger.error(
                        "Camera process (pid=%s) died! "
                        "Navigation continues without new detections.", p.pid)
                    if _gui_state:
                        _gui_state.add_error('watchdog', 'Camera process crashed')
                else:
                    logger.critical(
                        "Nav process (pid=%s) died! Shutting down.", p.pid)
                    shutdown_event.set()
                    try:
                        GLib.idle_add(Gtk.main_quit)
                    except Exception:
                        pass
                    return
        time.sleep(1.0)


def main_multiprocess():
    """Launch GUI in the main process with Nav and Camera as child processes.

    Uses 'spawn' start method to avoid fork issues with GTK and rclpy.
    Spawns Nav and Camera processes (each creates its own rclpy context),
    starts a SHM poll thread that reads struct-based SHM directly,
    starts a watchdog thread, initializes GTK, and enters the main loop.
    """
    global _shutdown_event, _gui_state, _processes
    import multiprocessing as mp

    # spawn avoids fork issues with GTK/rclpy
    mp.set_start_method('spawn', force=True)

    from target_nav.state.shared_state import SharedState

    # Clean up stale shared memory from previous crashed runs.
    from target_nav.config import (
        SHM_CAMERA_PATH, SHM_DETECTIONS_PATH,
        SHM_CALIBRATION_PATH, SHM_SETTINGS_PATH,
    )
    from target_nav.ipc.shm_struct import (
        NavShmReader, LaserShmReader, DetShmReader, CmdRingBuffer,
        DoubleBuffer, NavSnapshot, cleanup_all_shm,
    )

    # Clean struct-based SHM files
    cleanup_all_shm()

    # Clean legacy SHM files
    for shm_path in [SHM_CAMERA_PATH, SHM_DETECTIONS_PATH,
                     SHM_CALIBRATION_PATH, SHM_SETTINGS_PATH]:
        try:
            os.remove(shm_path)
            logger.info("Cleaned stale SHM: %s", shm_path)
        except FileNotFoundError:
            pass
        except Exception as e:
            logger.warning("Could not clean SHM %s: %s", shm_path, e)

    _shutdown_event = mp.Event()
    _gui_state = SharedState()

    atexit.register(_cleanup_multiprocess)
    signal.signal(signal.SIGINT, _signal_handler_mp)
    signal.signal(signal.SIGTERM, _signal_handler_mp)

    logger.info("=" * 60)
    logger.info("V2N Robot Control - 3-PROCESS MODE")
    logger.info("  Core 0: GUI (pure GTK — reads struct SHM directly)")
    logger.info("  Core 1: ROS2 Navigation (NavNode)")
    logger.info("  Core 2: Camera + DRP-AI Detection (CameraNode)")
    logger.info("=" * 60)

    # Pin GUI process to core 0
    try:
        os.sched_setaffinity(0, {0})
        logger.info("GUI pinned to core 0")
    except Exception:
        pass

    # Spawn child processes (each creates its own rclpy context)
    from target_nav.app.nav_node import main as nav_node_main
    from target_nav.app.camera_node import main as camera_node_main

    nav_p = mp.Process(target=nav_node_main, name="NavProcess", daemon=False)
    nav_p.start()
    _processes.append(nav_p)
    logger.info("Nav process started (pid=%s)", nav_p.pid)

    cam_p = mp.Process(target=camera_node_main, name="CameraProcess", daemon=False)
    cam_p.start()
    _processes.append(cam_p)
    logger.info("Camera process started (pid=%s)", cam_p.pid)

    # Start watchdog thread
    threading.Thread(
        target=_watchdog_mp, args=(_shutdown_event,),
        name="Watchdog", daemon=True).start()

    if _shutdown_event.is_set():
        return

    # Initialize GTK (no ROS2 in this process — pure GTK)
    ok = Gtk.init_check(sys.argv)[0] if callable(getattr(Gtk, 'init_check', None)) else True
    if not ok:
        logger.error("GTK init FAILED.")
        _shutdown_event.set()
        return

    apply_theme()

    # Clean init
    import numpy as np
    _gui_state.nav.clear_nav_target_map()
    _gui_state.nav.set_lidar_at_target(float('inf'))
    _gui_state.nav.set_nav_state("IDLE", None)
    _gui_state.nav.set_current_cmd_vel(0.0, 0.0, 0.0)
    _gui_state.sensors.set_robot_pose(0.0, 0.0, 0.0)
    _gui_state.sensors.set_laser(np.empty((0, 2), dtype=np.float32))
    _gui_state.detection.set_camera(None, [], "", False)
    _gui_state.detection.set_detector_mode("Initializing")

    # --- Struct-based SHM readers (read directly from Nav and Camera) ---
    _nav_reader = NavShmReader()
    _laser_reader = LaserShmReader()
    _det_reader = DetShmReader()
    _cmd_ring = CmdRingBuffer.create()  # GUI is the producer (writer)

    # Command type constants for the ring buffer
    from target_nav.ipc.shm_struct import CMD_GO, CMD_STOP, CMD_SET_PARAM, CMD_TEST_MOTOR

    # Wire send_ros_command to write to CmdRingBuffer (nav process polls)
    def _send_cmd(cmd_dict):
        cmd_type_str = cmd_dict.get('type', '')
        # Map string command types to integer constants
        cmd_map = {
            'go': CMD_GO,
            'navigate': CMD_GO,
            'stop': CMD_STOP,
            'set_nav_param': CMD_SET_PARAM,
            'set_param': CMD_SET_PARAM,
            'settings_changed': CMD_SET_PARAM,
            'drpai_restart': CMD_SET_PARAM,
            'test_motor': CMD_TEST_MOTOR,
            'arduino_cmd': CMD_SET_PARAM,
            'reset_odom': CMD_SET_PARAM,
            'reset_nav_params': CMD_SET_PARAM,
        }
        cmd_int = cmd_map.get(cmd_type_str.lower())
        if cmd_int is None:
            logger.warning("Unknown command type: %s", cmd_type_str)
            return
        payload = json.dumps(cmd_dict).encode('utf-8')
        _cmd_ring.push(cmd_int, payload)
    _gui_state.send_ros_command = _send_cmd

    # Wire settings publisher
    def _publish_settings():
        payload = json.dumps({'type': 'settings_changed'}).encode('utf-8')
        _cmd_ring.push(CMD_SET_PARAM, payload)
    _gui_state._settings_pub = type('Pub', (), {'publish': lambda self, msg: _publish_settings()})()

    # Wire DRP-AI restart publisher
    def _publish_drpai_restart():
        payload = json.dumps({'type': 'drpai_restart'}).encode('utf-8')
        _cmd_ring.push(CMD_SET_PARAM, payload)
    _gui_state._publish_drpai_restart = _publish_drpai_restart

    # SHM frame reader for camera panel (reads from C++ DRP-AI SHM)
    import mmap as mmap_mod
    import struct as struct_mod
    _shm_fd = [None]
    _shm_mmap = [None]

    def _read_shm_frame():
        """Read camera frame from C++ DRP-AI shared memory."""
        try:
            if _shm_mmap[0] is None:
                if not os.path.exists(SHM_CAMERA_PATH):
                    return None
                _shm_fd[0] = os.open(SHM_CAMERA_PATH, os.O_RDONLY)
                size = os.fstat(_shm_fd[0]).st_size
                if size < 24:
                    os.close(_shm_fd[0])
                    _shm_fd[0] = None
                    return None
                try:
                    _shm_mmap[0] = mmap_mod.mmap(_shm_fd[0], size, access=mmap_mod.ACCESS_READ)
                except Exception:
                    os.close(_shm_fd[0])
                    _shm_fd[0] = None
                    return None
            _shm_mmap[0].seek(0)
            header = _shm_mmap[0].read(24)
            w, h, seq_before, ts = struct_mod.unpack('<IIqq', header)
            if w == 0 or h == 0 or seq_before == 0 or w > 2048 or h > 2048:
                return None
            data = _shm_mmap[0].read(w * h * 3)
            _shm_mmap[0].seek(8)
            seq_after = struct_mod.unpack('<q', _shm_mmap[0].read(8))[0]
            if seq_after != seq_before:
                return None
            return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
        except Exception:
            return None

    # GUI timer polls SHM data directly from Nav and Camera processes
    def _gui_poll_shm():
        """Read struct SHM from Nav and Camera processes, update GUI state.

        Reads from 3 struct-based SHM channels (no Bridge process):
        - Nav:        Navigation state struct (from NavShmWriter in nav process)
        - Laser:      LiDAR points + robot pose (from LaserShmWriter in nav process)
        - Detections: Detection structs (from DetShmWriter in camera process)
        Also reads camera frame directly from C++ DRP-AI SHM.
        """
        if _shutdown_event.is_set():
            return False

        # --- Channel 1: Nav state (struct, not JSON) ---
        snap = _nav_reader.read()
        if snap is not None:
            nav = _gui_state.nav
            nav.set_nav_state(snap.nav_state_name)
            tst = snap.time_since_target
            if tst < 999.0:
                nav.set_last_target_time(time.time() - tst)
            st_time = snap.search_time
            if st_time > 0:
                nav.set_search_start_time(time.time() - st_time)
            _lat = snap.lidar_at_target
            nav.set_lidar_at_target(float('inf') if _lat >= 900.0 else _lat)
            trx, try_ = snap.target_robot_x, snap.target_robot_y
            if trx != 0.0 or try_ != 0.0:
                nav.set_target_robot_xy(trx, try_)
            else:
                nav.clear_target_robot_xy()
            nav.set_obstacle(snap.obstacle_ahead, snap.obstacle_dist)
            nav.set_current_cmd_vel(snap.vx, snap.vy, snap.wz)
            # Nav target in map frame
            nmx, nmy = snap.nav_target_map_x, snap.nav_target_map_y
            if nmx != 0.0 or nmy != 0.0:
                nav.set_nav_target_map(nmx, nmy)
            # Detector mode
            if snap.detector_mode:
                _gui_state.detection.set_detector_mode(snap.detector_mode)

        # --- Channel 2: LiDAR data (struct, not JSON) ---
        laser = _laser_reader.read()
        if laser is not None:
            points, px, py, pt, hz, n_raw = laser
            # Convert list of (x,y) tuples to numpy array for map_panel
            arr = np.array(points, dtype=np.float32).reshape(-1, 2) if points else np.empty((0, 2), dtype=np.float32)
            _gui_state.sensors.set_laser(arr)
            _gui_state.sensors.set_robot_pose(px, py, pt)

        # --- Channel 3: Detections (struct, not JSON) ---
        det_result = _det_reader.read()
        if det_result is not None:
            timestamp, dets, info = det_result
            from target_nav.detectors.detection import Det
            det_objects = [Det(
                class_name=d.class_name,
                confidence=d.confidence,
                bbox=(d.bbox_x1, d.bbox_y1, d.bbox_x2, d.bbox_y2),
                distance=d.distance,
                angle=d.angle,
                bbox_clipped=d.bbox_clipped,
            ) for d in dets]
            frame = _read_shm_frame()
            _gui_state.detection.set_camera(
                frame, det_objects, info, fresh_detection=True)
        else:
            # No new detections — still refresh camera frame for live video
            frame = _read_shm_frame()
            if frame is not None:
                _gui_state.detection.set_camera(
                    frame, [], "", fresh_detection=False)

        return True  # keep timer running

    # SHM polling runs in a lightweight daemon thread.
    # Thread safety: SharedState stores use RLock — safe for concurrent
    # read (GTK) + write (this thread). SHM reads are fast (<1ms mmap)
    # and release the GIL during I/O, so GTK main loop is never blocked.
    _shm_poll_stop = threading.Event()

    def _shm_poll_thread():
        logger.info("SHM poll thread running (20Hz)")
        while not _shm_poll_stop.is_set() and not _shutdown_event.is_set():
            try:
                _gui_poll_shm()
            except Exception as e:
                logger.debug("SHM poll error: %s", e)
            _shm_poll_stop.wait(0.05)  # 20Hz, interruptible
        logger.info("SHM poll thread stopped")

    shm_thread = threading.Thread(
        target=_shm_poll_thread, name="ShmPoll", daemon=True)
    shm_thread.start()

    try:
        win = MainGUI(_gui_state)
        win.show_all()
        logger.info("GUI running (3-process, struct SHM)")
        Gtk.main()
    except Exception as e:
        logger.error("GUI error: %s", e)
    finally:
        # Clean shutdown sequence:
        # 1. Stop poll thread (no more SHM reads)
        # 2. Close all SHM resources
        # 3. Signal child processes to exit
        # 4. atexit handler (_cleanup_multiprocess) kills any survivors
        logger.info("GUI shutting down...")

        # Stop SHM poll thread
        _shm_poll_stop.set()
        shm_thread.join(timeout=2.0)
        if shm_thread.is_alive():
            logger.warning("SHM poll thread didn't stop in 2s")

        # Close struct SHM readers/writers
        try:
            _nav_reader.close()
        except Exception:
            pass
        try:
            _laser_reader.close()
        except Exception:
            pass
        try:
            _det_reader.close()
        except Exception:
            pass
        try:
            _cmd_ring.close()
        except Exception:
            pass

        # Close camera frame SHM
        if _shm_mmap[0]:
            try:
                _shm_mmap[0].close()
            except Exception:
                pass
        if _shm_fd[0] and _shm_fd[0] >= 0:
            try:
                os.close(_shm_fd[0])
            except Exception:
                pass

        # Signal child processes
        _shutdown_event.set()
        _gui_state.close()

        # Clean SHM files so next startup is fresh
        cleanup_all_shm()
        # Also clean camera SHM
        try:
            os.remove(SHM_CAMERA_PATH)
        except (FileNotFoundError, Exception):
            pass

        logger.info("GUI shutdown complete")


# ==================================================================
# Entry point
# ==================================================================

def main():
    """Application entry point -- configure logging and launch 3-process mode."""
    from target_nav.utils.log import setup_logging
    setup_logging()

    logger.info("Multiprocess mode (3-process, struct SHM)")
    main_multiprocess()


if __name__ == '__main__':
    main()
