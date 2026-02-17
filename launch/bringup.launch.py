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
    # Get package directory
    pkg_dir = get_package_share_directory('bowling_target_nav')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'v2n_robot.urdf')

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

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

    use_robot_state_pub_arg = DeclareLaunchArgument(
        'use_robot_state_publisher',
        default_value='true',
        description='Publish robot URDF for TF and visualization'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (for rosbag replay or Gazebo)'
    )

    # Robot state publisher (publishes URDF TF: base_link -> laser, wheels, etc.)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        arduino_port_arg,
        lidar_frame_arg,
        use_robot_state_pub_arg,
        use_sim_time_arg,
        robot_state_publisher,
        rplidar_launch,
        arduino_node,
        odometry_node,
    ])
