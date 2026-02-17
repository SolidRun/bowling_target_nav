"""
Rosbag Recording Launch File
=============================

Records key topics for offline analysis and replay.

Usage:
    # Record all default topics
    ros2 launch bowling_target_nav record.launch.py

    # Custom output directory
    ros2 launch bowling_target_nav record.launch.py bag_dir:=/path/to/bags

    # Record with compression
    ros2 launch bowling_target_nav record.launch.py compress:=true
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Default bag directory
    default_bag_dir = os.path.join(os.path.expanduser('~'), 'rosbags')

    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir',
        default_value=default_bag_dir,
        description='Directory to save rosbag files'
    )

    compress_arg = DeclareLaunchArgument(
        'compress',
        default_value='false',
        description='Enable zstd compression'
    )

    # Topics to record
    topics = [
        '/scan',                  # LiDAR scans
        '/cmd_vel',               # Velocity commands
        '/odom',                  # Odometry
        '/target_pose',           # Vision target pose
        '/target_detection',      # Vision detection details
        '/arduino/odom_raw',      # Raw Arduino odometry
        '/arduino/status',        # Arduino connection status
        '/diagnostics',           # System diagnostics
        '/tf',                    # Transform tree
        '/tf_static',             # Static transforms
    ]

    # Build ros2 bag record command
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    cmd = [
        'ros2', 'bag', 'record',
        '--output', [LaunchConfiguration('bag_dir'), f'/v2n_{timestamp}'],
        '--max-bag-duration', '300',  # Split every 5 minutes
    ] + topics

    record_process = ExecuteProcess(
        cmd=cmd,
        output='screen',
    )

    return LaunchDescription([
        bag_dir_arg,
        compress_arg,
        record_process,
    ])
