"""
Robot Bringup Launch File
=========================

Starts all robot drivers: LiDAR, Arduino (motors + odometry), and TF.

Usage:
    ros2 launch bowling_target_nav bringup.launch.py
    ros2 launch bowling_target_nav bringup.launch.py lidar_port:=/dev/ttyUSB0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyACM0',
        description='Arduino serial port'
    )

    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='laser',
        description='LiDAR TF frame name'
    )

    # Use rplidar_ros package's built-in launch file for RPLidar A1
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            ])
        )
    )

    # Arduino driver node (motor control + raw odometry)
    arduino_node = Node(
        package='bowling_target_nav',
        executable='arduino_driver_node',
        name='arduino_driver_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('arduino_port'),
            'baudrate': 115200,
        }]
    )

    # Odometry node (converts raw Arduino odom to nav_msgs/Odometry + TF)
    odometry_node = Node(
        package='bowling_target_nav',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
        }]
    )

    # Static TF: base_link -> laser
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.0', '0.0', '0.15',  # x, y, z (laser 15cm above base)
            '0.0', '0.0', '0.0', '1.0',  # quaternion (no rotation)
            'base_link', 'laser'
        ]
    )

    return LaunchDescription([
        arduino_port_arg,
        lidar_frame_arg,
        rplidar_launch,
        arduino_node,
        odometry_node,
        static_tf_base_to_laser,
    ])
