"""
Autonomous Exploration + Target Push Launch File
=================================================

Starts the full autonomous system:
    1. Bringup (LiDAR, Arduino, Odometry, TF)
    2. Cartographer SLAM (builds map while driving)
    3. Vision node (camera + YOLO detection with video display)
    4. Autonomous explorer (scan 360, wander, approach, push)

Usage:
    # Full system with camera display on screen
    ros2 launch bowling_target_nav autonomous.launch.py

    # Without Cartographer (lighter)
    ros2 launch bowling_target_nav autonomous.launch.py use_slam:=false

    # Custom speeds
    ros2 launch bowling_target_nav autonomous.launch.py linear_speed:=0.15
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('bowling_target_nav')

    # --- Launch Arguments ---
    use_slam_arg = DeclareLaunchArgument(
        'use_slam', default_value='true',
        description='Enable Cartographer SLAM mapping')

    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed', default_value='0.18',
        description='Wander forward speed (m/s)')

    target_class_arg = DeclareLaunchArgument(
        'target_class', default_value='bowling-pins',
        description='Object class to track')

    detector_type_arg = DeclareLaunchArgument(
        'detector_type', default_value='yolo',
        description='Detector type: yolo or binary')

    # --- 1. Bringup (LiDAR + Arduino + Odom + TF) ---
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'bringup.launch.py')
        )
    )

    # --- 2. Cartographer SLAM (optional) ---
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'mapping.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )

    # --- 3. Vision Node (camera + bowling-pins ONLY detection + video) ---
    vision_node = Node(
        package='bowling_target_nav',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{
            'detector_type': LaunchConfiguration('detector_type'),
            'target_class': LaunchConfiguration('target_class'),
            'conf_threshold': 0.4,
            'show_video': True,
            'enable_visualization': True,
            'publish_rate': 5.0,
            'frame_width': 640,
            'frame_height': 480,
        }],
    )

    # --- 4. Autonomous Explorer ---
    explorer_node = Node(
        package='bowling_target_nav',
        executable='autonomous_explorer',
        name='autonomous_explorer',
        output='screen',
        parameters=[{
            'linear_speed': LaunchConfiguration('linear_speed'),
        }],
    )

    return LaunchDescription([
        use_slam_arg,
        linear_speed_arg,
        target_class_arg,
        detector_type_arg,
        bringup_launch,
        mapping_launch,
        vision_node,
        explorer_node,
    ])
