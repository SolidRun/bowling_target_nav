"""
SLAM Launch File (Bringup + Mapping)
====================================

Launches everything needed for SLAM mapping:
- RPLidar driver
- Arduino driver + odometry
- Cartographer SLAM
- TF transforms

Usage:
    ros2 launch bowling_target_nav slam.launch.py
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

    # Launch arguments
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyACM0',
        description='Arduino serial port'
    )

    show_map_arg = DeclareLaunchArgument(
        'show_map',
        default_value='false',
        description='Show map on screen using OpenCV'
    )

    # Map viewer node (OpenCV-based, for V2N screen)
    map_viewer_node = Node(
        package='bowling_target_nav',
        executable='map_viewer_node',
        name='map_viewer_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_map'))
    )

    # Include bringup launch (uses rplidar_ros package's A1 launch)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'arduino_port': LaunchConfiguration('arduino_port'),
        }.items()
    )

    # Include mapping launch
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false',
        }.items()
    )

    return LaunchDescription([
        arduino_port_arg,
        show_map_arg,
        bringup_launch,
        mapping_launch,
        map_viewer_node,
    ])
