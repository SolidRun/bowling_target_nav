"""Route GUI commands to the appropriate state/ROS actions.

Dispatches command dictionaries from the GUI to the navigation state machine,
settings store, and ROS publishers. In 3-process mode, the GUI writes
commands to a CmdRingBuffer (lock-free SPSC struct SHM), the Nav process
polls CmdRingBuffer.pop() at 20 Hz and calls dispatch_command().

Supported command types (cmd_dict['type'] -> payload keys):
    go               -- Start navigation. Sets GO flag, resets bbox filter.
    stop             -- Stop navigation. Sets STOP flag.
    stop_robot       -- Emergency stop: publishes zero Twist + sets STOP flag.
    set_nav_param    -- Update one nav param. Keys: 'attr', 'value'.
    reset_nav_params -- Reset all nav params to DEFAULT_NAV_PARAMS.
    arduino_cmd      -- Send raw string to Arduino. Key: 'command' (str).
    reset_odom       -- Publish Empty on /reset_odom topic.
    test_motor       -- Test one motor at PWM. Keys: 'motor' (str), 'pwm' (int).
    test_motor_twist -- Publish Twist directly. Keys: 'vx', 'vy', 'wz' (float).

Runs in: Nav process (Core 1), called from NavNode._poll_commands().

Related modules:
    ipc/shm_struct.py      -- CmdRingBuffer for GUI -> Nav commands.
    gui/settings_tabs/     -- builds cmd_dicts sent here.
"""

from target_nav.config import DEFAULT_NAV_PARAMS
from target_nav.utils.log import get_logger

logger = get_logger('state.command_dispatcher')


def dispatch_command(cmd_dict, state, node):
    """Route a command dictionary to the appropriate handler.

    Args:
        cmd_dict: Dict with 'type' key and command-specific payload.
        state: SharedState instance (provides nav, detection stores).
        node: ROS2 node with publishers (cmd_vel_pub, arduino_cmd_pub,
              reset_odom_pub). Can be None if ROS is not available.

    Returns:
        'bbox_reset' if the command triggers a bbox reset (GO command),
        None otherwise.
    """
    cmd_type = cmd_dict.get('type', '')

    if cmd_type == 'go':
        state.nav.request_go()
        state.detection.request_bbox_reset()
        return 'bbox_reset'

    elif cmd_type == 'stop':
        state.nav.request_stop()

    elif cmd_type == 'stop_robot':
        # Emergency: zero velocity + stop flag
        if node is not None:
            from geometry_msgs.msg import Twist
            node.cmd_vel_pub.publish(Twist())
        state.nav.request_stop()

    elif cmd_type == 'set_nav_param':
        attr = cmd_dict.get('attr')
        value = cmd_dict.get('value')
        if attr is not None and value is not None:
            state.detection.set_nav_param(attr, value)

    elif cmd_type == 'reset_nav_params':
        for key, value in DEFAULT_NAV_PARAMS.items():
            state.detection.set_nav_param(key, value)

    elif cmd_type == 'arduino_cmd':
        # Tools tab sends 'data' key, accept both 'data' and 'command' for compat
        command = cmd_dict.get('data', '') or cmd_dict.get('command', '')
        if node is not None and command:
            from std_msgs.msg import String
            msg = String()
            msg.data = command
            node.arduino_cmd_pub.publish(msg)

    elif cmd_type == 'reset_odom':
        if node is not None:
            from std_msgs.msg import Empty
            node.reset_odom_pub.publish(Empty())

    elif cmd_type == 'test_motor':
        motor = cmd_dict.get('motor', '')
        pwm = cmd_dict.get('pwm', 0)
        if node is not None and motor:
            from std_msgs.msg import String
            msg = String()
            msg.data = f'TMOTOR,{motor},{pwm}'  # firmware expects TMOTOR, not MOTOR
            node.arduino_cmd_pub.publish(msg)

    elif cmd_type == 'test_motor_twist':
        vx = cmd_dict.get('vx', 0.0)
        vy = cmd_dict.get('vy', 0.0)
        wz = cmd_dict.get('wz', 0.0)
        if node is not None:
            from geometry_msgs.msg import Twist
            cmd = Twist()
            cmd.linear.x = float(vx)
            cmd.linear.y = float(vy)
            cmd.angular.z = float(wz)
            node.cmd_vel_pub.publish(cmd)

    elif cmd_type == 'settings_changed':
        # GUI saved new settings — reload from disk into nav process state
        try:
            state.detection._load_calibration()
            state.detection.mark_calibration_dirty()
            logger.info("Settings reloaded from disk (settings_changed command)")
        except Exception as e:
            logger.warning("Settings reload failed: %s", e)
        # Forward to camera process via ROS topic (camera subscribes to this)
        if node is not None:
            from std_msgs.msg import String as _Str
            msg = _Str()
            msg.data = 'settings_changed'
            try:
                node.create_publisher(_Str, '/settings_changed', 1).publish(msg)
            except Exception:
                pass

    elif cmd_type == 'drpai_restart':
        # GUI requests DRP-AI restart — forward to camera node
        if node is not None:
            from std_msgs.msg import String as _Str
            msg = _Str()
            msg.data = 'restart'
            try:
                node.create_publisher(_Str, '/drpai_restart', 1).publish(msg)
            except Exception:
                pass

    else:
        logger.warning("Unknown command type: %s", cmd_type)

    return None
