"""
=============================================================================
Bowling Target Navigation Launch File
=============================================================================

Launches the vision and navigation nodes for autonomous bowling pin approach.

Components:
    1. vision_node - Camera capture and YOLO detection
    2. target_follower_node - Robot motion control

Usage:
    # Basic launch (uses default params)
    ros2 launch bowling_target_nav bowling_target_nav.launch.py

    # With custom parameters file
    ros2 launch bowling_target_nav bowling_target_nav.launch.py \
        params_file:=/path/to/custom_params.yaml

    # Vision only (no robot motion)
    ros2 launch bowling_target_nav bowling_target_nav.launch.py \
        enable_follower:=false

    # With Nav2 mode (requires Nav2 stack running)
    ros2 launch bowling_target_nav bowling_target_nav.launch.py \
        follower_mode:=nav2

Prerequisites:
    1. Camera connected (/dev/video0)
    2. For Nav2 mode: Nav2 stack running (bringup.launch.py with enable_nav2:=true)

Topics Published:
    /target_pose - PoseStamped of detected target
    /target_detection - JSON string with detection details
    /cmd_vel - Robot velocity commands (direct mode only)

=============================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_share = FindPackageShare('bowling_target_nav')

    # Default params file
    default_params = PathJoinSubstitution([
        pkg_share, 'config', 'target_nav_params.yaml'
    ])

    # Default model path
    default_model = PathJoinSubstitution([
        pkg_share, 'models', 'bowling_yolov8m.onnx'
    ])

    return LaunchDescription([
        # =====================================================================
        # Launch Arguments
        # =====================================================================
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to parameters YAML file'
        ),

        DeclareLaunchArgument(
            'enable_follower',
            default_value='true',
            description='Enable target follower node (set false for vision-only testing)'
        ),

        DeclareLaunchArgument(
            'follower_mode',
            default_value='direct',
            description='Follower mode: "direct" or "nav2"'
        ),

        DeclareLaunchArgument(
            'target_class',
            default_value='Pins',
            description='Object class to track (Pins, Ball, Pin+Ball, Sweep)'
        ),

        DeclareLaunchArgument(
            'model_path',
            default_value=default_model,
            description='Path to YOLO ONNX model'
        ),

        DeclareLaunchArgument(
            'enable_gui',
            default_value='false',
            description='Enable GUI display (requires Wayland on V2N)'
        ),

        DeclareLaunchArgument(
            'enable_arduino',
            default_value='false',
            description='Enable Arduino driver node (alternative to my_package arduino nodes)'
        ),

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Arduino serial port'
        ),

        # =====================================================================
        # Vision Node
        # =====================================================================
        Node(
            package='bowling_target_nav',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'model_path': LaunchConfiguration('model_path'),
                    'target_class': LaunchConfiguration('target_class'),
                }
            ],
        ),

        # =====================================================================
        # Target Follower Node
        # =====================================================================
        Node(
            package='bowling_target_nav',
            executable='target_follower_node',
            name='target_follower_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_follower')),
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'mode': LaunchConfiguration('follower_mode'),
                }
            ],
        ),

        # =====================================================================
        # GUI Node (Optional - for V2N display)
        # =====================================================================
        Node(
            package='bowling_target_nav',
            executable='target_gui_node',
            name='target_gui_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_gui')),
        ),

        # =====================================================================
        # Arduino Driver Node (Optional - professional serial interface)
        # =====================================================================
        Node(
            package='bowling_target_nav',
            executable='arduino_driver_node',
            name='arduino_driver',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_arduino')),
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'max_linear_speed': 0.436,   # m/s at PWM 255
                'max_angular_speed': 2.18,   # rad/s at PWM 255
                'command_rate': 20.0,        # Hz (must be ≥5 for 200ms watchdog)
                'command_timeout': 0.5,      # seconds
                'enable_diagnostics': True,
            }],
        ),
    ])
