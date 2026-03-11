"""ROS+Navigation process — runs on a dedicated CPU core.

Owns:
    - ROS2 node (map, scan, TF, cmd_vel publishers)
    - Navigator (obstacle avoidance, target tracking, search)
    - Local SharedState for internal communication

Syncs with GUI process via IPCHub:
    - Writes: map, pose, laser, nav state, diagnostics
    - Reads: commands (go/stop/settings), detections (from camera)
"""

import os
import signal
import threading
import time
import traceback
from queue import Empty as QueueEmpty

_TAG = "[ROS Process]"


def _log(msg):
    print(f"{_TAG} {msg}", flush=True)


def _sync_outbound(hub, local_state):
    """Thread: sync local state -> IPCHub (ROS -> GUI).

    Runs at ~50Hz.  Copies pose every tick (cheap), but only copies the
    SLAM map image (~12 MB) and laser points when their update counts
    change.  Also syncs the nav state snapshot and diagnostics.

    Shuts down after 50 consecutive errors to avoid infinite loops.

    Args:
        hub: IPCHub instance shared with GUI and Camera processes.
        local_state: SharedState instance local to the ROS process.
    """
    last_map_count = 0
    last_scan_count = 0
    consecutive_errors = 0

    while not hub.shutdown.is_set():
        try:
            # Robot pose (cheap, always sync)
            x, y, theta = local_state.sensors.get_robot_pose()
            hub.write_pose(x, y, theta)

            # Map (only copy when new data — check count first to avoid
            # expensive ~12MB image copy on every tick)
            map_count = local_state.sensors.get_map_count()
            if map_count > last_map_count:
                map_img, map_info, _ = local_state.sensors.get_map()
                if map_img is not None:
                    hub.write_map(map_img, map_info)
                last_map_count = map_count

            # Laser points (only when new data)
            points, scan_count, laser_time = local_state.sensors.get_laser()
            if scan_count > last_scan_count:
                hub.write_laser(points, laser_time)
                last_scan_count = scan_count

            # Nav state snapshot
            snap = local_state.nav.get_gui_snapshot()
            hub.write_nav_state(
                state_str=snap['nav_state'],
                nav_target=snap['nav_target'],
                nav_target_map=snap['nav_target_map'],
                cmd_vel=snap['cmd_vel'],
                time_since_target=snap['time_since_target'],
                search_time=snap['search_time'],
                obstacle_dist=snap['obstacle_dist'],
                obstacle_ahead=snap['obstacle_ahead'],
                lidar_at_target=snap.get('lidar_at_target', float('inf')))

            # Diagnostics
            diag = local_state.sensors.get_diagnostics()
            if diag:
                hub.write_diagnostics(**diag)

            consecutive_errors = 0

        except Exception as e:
            consecutive_errors += 1
            if consecutive_errors <= 3:
                hub.add_error('ros_sync_out', str(e))
            elif consecutive_errors == 10:
                hub.add_error('ros_sync_out', f'Repeated errors ({consecutive_errors}x): {e}')
            if consecutive_errors > 50:
                _log(f"sync_outbound: too many errors ({consecutive_errors}), shutting down")
                hub.shutdown.set()
                return

        time.sleep(0.02)  # 50Hz


def _sync_inbound(hub, local_state, ros_node_ref):
    """Thread: sync IPCHub -> local state (GUI/Camera -> ROS).

    Runs at ~50Hz.  Handles four responsibilities:
        1. Emergency stop via mp.Event (never dropped, even if queue full).
        2. Command dispatch from GUI (go, stop, motor test, nav params, etc.).
        3. Detection forwarding from Camera process to local state.
        4. Calibration/settings reload when settings_changed_ros fires.

    Shuts down after 50 consecutive errors.

    Args:
        hub: IPCHub instance shared with GUI and Camera processes.
        local_state: SharedState instance local to the ROS process.
        ros_node_ref: Single-element list holding the ROS node (mutable
            reference so the thread sees the node after it is created).
    """
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String, Empty as EmptyMsg

    consecutive_errors = 0

    while not hub.shutdown.is_set():
        try:
            # --- Emergency stop (mp.Event — never dropped) ---
            if hub.emergency_stop.is_set():
                hub.emergency_stop.clear()
                node = ros_node_ref[0]
                if node and hasattr(node, 'cmd_vel_pub'):
                    node.cmd_vel_pub.publish(Twist())
                local_state.nav.request_stop()
                _log("Emergency stop received via event")

            # --- Commands from GUI ---
            while True:
                try:
                    cmd = hub.cmd_queue.get_nowait()
                except QueueEmpty:
                    break
                except OSError:
                    break
                cmd_type = cmd.get('type', '')
                node = ros_node_ref[0]

                if cmd_type == 'go':
                    local_state.nav.request_go()
                    hub.bbox_reset_cam.set()  # Signal camera to reset bbox tracker
                elif cmd_type == 'stop':
                    local_state.nav.request_stop()
                elif cmd_type == 'stop_robot':
                    if node and hasattr(node, 'cmd_vel_pub'):
                        node.cmd_vel_pub.publish(Twist())
                elif cmd_type == 'set_nav_param':
                    attr = cmd.get('attr', '')
                    value = cmd.get('value')
                    local_state.detection.set_nav_param(attr, value)
                    if node and hasattr(node, 'navigator') and hasattr(node.navigator, attr):
                        setattr(node.navigator, attr, value)
                elif cmd_type == 'set_robot_half_width':
                    value = cmd.get('value', 0.13)
                    if node and hasattr(node, 'navigator'):
                        node.navigator.robot_half_width = value
                elif cmd_type == 'reset_nav_params':
                    from bowling_target_nav.state.detection_store import DEFAULT_NAV_PARAMS
                    for k, v in DEFAULT_NAV_PARAMS.items():
                        local_state.detection.set_nav_param(k, v)
                    if node and hasattr(node, 'navigator'):
                        for k, v in DEFAULT_NAV_PARAMS.items():
                            if hasattr(node.navigator, k):
                                setattr(node.navigator, k, v)
                elif cmd_type == 'arduino_cmd':
                    data = cmd.get('data', '')
                    if node and hasattr(node, 'arduino_cmd_pub'):
                        msg = String()
                        msg.data = data
                        node.arduino_cmd_pub.publish(msg)
                elif cmd_type == 'reset_odom':
                    if node and hasattr(node, 'reset_odom_pub'):
                        node.reset_odom_pub.publish(EmptyMsg())
                elif cmd_type == 'test_motor':
                    if node and hasattr(node, 'cmd_vel_pub'):
                        twist = Twist()
                        direction = cmd.get('direction', '')
                        speed = cmd.get('speed', 0.1)
                        if direction == 'forward':
                            twist.linear.x = speed
                        elif direction == 'backward':
                            twist.linear.x = -speed
                        elif direction == 'left':
                            twist.linear.y = speed
                        elif direction == 'right':
                            twist.linear.y = -speed
                        elif direction == 'rotate_left':
                            twist.angular.z = speed
                        elif direction == 'rotate_right':
                            twist.angular.z = -speed
                        node.cmd_vel_pub.publish(twist)

            # --- Detections from Camera process ---
            dets, info, ts = hub.read_detections()
            det_age = time.time() - ts if ts > 0 else 999.0
            if ts > 0:
                # Always mark fresh when camera process has written data —
                # the camera process already manages timestamps correctly.
                # Using det_age < 0.5 was too tight and caused detections
                # to appear stale during normal DRP-AI inference gaps.
                local_state.detection.set_camera(
                    None, dets, info, fresh_detection=(det_age < 2.0))

            # --- Settings reload ---
            if hub.settings_changed_ros.is_set():
                hub.settings_changed_ros.clear()
                _log("Reloading calibration from disk...")
                local_state.detection._load_calibration()
                local_state.detection._calibration_dirty = True
                node = ros_node_ref[0]
                if node and hasattr(node, 'navigator'):
                    np_dict = local_state.detection.get_nav_params()
                    nav = node.navigator
                    for k, v in np_dict.items():
                        if hasattr(nav, k):
                            setattr(nav, k, v)
                    nav.robot_half_width = local_state.detection.get_robot_half_width()
                _log("Calibration reloaded")

            consecutive_errors = 0

        except Exception as e:
            consecutive_errors += 1
            if consecutive_errors <= 3:
                hub.add_error('ros_sync_in', str(e))
            elif consecutive_errors == 10:
                hub.add_error('ros_sync_in', f'Repeated errors ({consecutive_errors}x): {e}')
            if consecutive_errors > 50:
                _log(f"sync_inbound: too many errors ({consecutive_errors}), shutting down")
                hub.shutdown.set()
                return

        time.sleep(0.02)  # 50Hz


def ros_process_main(hub, core_id=1):
    """ROS process entry point.

    Lifecycle:
        1. Pin to CPU core, ignore SIGINT (parent handles signals).
        2. Create a local SharedState and patch it as the module singleton
           so that MainGuiNode and its navigator use process-local state.
        3. Initialize rclpy and create the ROS node.
        4. Start outbound and inbound sync daemon threads.
        5. Spin the ROS node until shutdown or too many errors.
        6. Clean up: join sync threads, close shm handles, destroy node.

    Args:
        hub: IPCHub instance (shared with other processes).
        core_id: CPU core to pin to (default 1).
    """
    _log("Starting...")

    # Pin to CPU core
    try:
        os.sched_setaffinity(0, {core_id})
        _log(f"Pinned to core {core_id}")
    except Exception:
        _log(f"Could not pin to core {core_id}")

    # Ignore SIGINT in child — parent handles it
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    ros_node_ref = [None]  # mutable reference for sync threads
    rclpy_initialized = False
    out_thread = in_thread = None

    try:
        import rclpy
        from bowling_target_nav.state.shared_state import SharedState

        # Create local state for this process BEFORE importing ros_node,
        # because ros_node does `from bowling_target_nav.state import state`
        # at module level. We must patch the singleton before that import.
        local_state = SharedState()
        import bowling_target_nav.state as state_module
        state_module.state = local_state

        from bowling_target_nav.threads.ros_node import MainGuiNode
        # Also patch the ros_node module's own binding
        import bowling_target_nav.threads.ros_node as ros_node_module
        ros_node_module.state = local_state

        rclpy.init()
        rclpy_initialized = True
        node = MainGuiNode()
        ros_node_ref[0] = node

        # Ensure navigator uses our local state
        node.navigator._state = local_state
        local_state._ros_node = node

        # Start sync threads
        out_thread = threading.Thread(
            target=_sync_outbound, args=(hub, local_state),
            name="ROSSyncOut", daemon=True)
        out_thread.start()

        in_thread = threading.Thread(
            target=_sync_inbound, args=(hub, local_state, ros_node_ref),
            name="ROSSyncIn", daemon=True)
        in_thread.start()

        _log("ROS node + sync threads started")

        # Spin ROS — resilient to transient errors
        spin_errors = 0
        while not hub.shutdown.is_set():
            try:
                rclpy.spin_once(node, timeout_sec=0.02)
                spin_errors = 0
            except Exception as e:
                if hub.shutdown.is_set():
                    break
                spin_errors += 1
                if spin_errors <= 3:
                    _log(f"spin_once error ({spin_errors}): {e}")
                    hub.add_error('ros_spin', str(e))
                if spin_errors > 10:
                    _log(f"Too many spin errors ({spin_errors}), exiting")
                    traceback.print_exc()
                    break
                time.sleep(0.1)  # brief pause before retry

    except Exception as e:
        _log(f"Fatal error: {e}")
        traceback.print_exc()
        hub.add_error('ros_process', str(e))
    finally:
        _log("Cleaning up...")
        # Signal shutdown and wait for sync threads before closing shm
        hub.shutdown.set()
        for t in (out_thread, in_thread):
            if t is not None:
                try:
                    t.join(timeout=1.0)
                except Exception:
                    pass
        # Close shared memory handles in child (without unlink)
        try:
            hub.close_child()
        except Exception:
            pass
        if ros_node_ref[0]:
            try:
                ros_node_ref[0].destroy_node()
            except Exception:
                pass
        if rclpy_initialized:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        _log("Stopped")
