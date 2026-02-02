"""
Cartographer Mapping Launch File
================================

Launches Cartographer SLAM for 2D LiDAR mapping.

Requirements:
    - LiDAR publishing to /scan
    - Odometry publishing to /odom
    - TF: odom -> base_link -> laser_frame

Usage:
    ros2 launch bowling_target_nav mapping.launch.py
    ros2 launch bowling_target_nav mapping.launch.py use_rviz:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('bowling_target_nav')

    # Configuration files
    cartographer_config = os.path.join(pkg_dir, 'config', 'cartographer.lua')
    rviz_config = os.path.join(pkg_dir, 'config', 'mapping.rviz')

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LaserScan topic name'
    )

    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_dir, 'config'),
            '-configuration_basename', 'cartographer.lua'
        ],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
        ]
    )

    # Occupancy grid node (converts Cartographer map to OccupancyGrid)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'resolution': 0.05},
            {'publish_period_sec': 1.0}
        ]
    )

    # RViz2 node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        scan_topic_arg,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])
