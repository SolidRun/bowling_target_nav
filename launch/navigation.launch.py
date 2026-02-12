"""
Nav2 Navigation Launch File
============================

Launches full Nav2 stack for autonomous navigation with obstacle avoidance.

Modes:
    - SLAM mode (default): Uses Cartographer for live mapping + navigation
    - Localization mode: Uses saved map + AMCL for localization

Requirements:
    - bringup.launch.py running (LiDAR, Arduino, Odometry)
    - For localization mode: saved map file

Usage:
    # SLAM + Navigation (no saved map needed)
    ros2 launch bowling_target_nav navigation.launch.py

    # Localization + Navigation (requires saved map)
    ros2 launch bowling_target_nav navigation.launch.py use_slam:=false map:=/path/to/map.yaml

    # Navigate to pose via command line:
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}}}}"
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('bowling_target_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Configuration files
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'v2n_robot.urdf')
    default_map = os.path.join(pkg_dir, 'maps', 'default_map.yaml')

    # Launch arguments
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Use SLAM for mapping (true) or localization on saved map (false)'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to map YAML file (only used when use_slam:=false)'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Nav2 parameters file'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 stack'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Robot state publisher (publishes URDF to /robot_description and TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # SLAM mode: Use Cartographer for mapping
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'mapping.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )

    # Localization mode: Use map_server + AMCL
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': False
        }],
        condition=UnlessCondition(LaunchConfiguration('use_slam'))
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        condition=UnlessCondition(LaunchConfiguration('use_slam'))
    )

    # Lifecycle manager for localization nodes
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': LaunchConfiguration('autostart'),
            'node_names': ['map_server', 'amcl']
        }],
        condition=UnlessCondition(LaunchConfiguration('use_slam'))
    )

    # Nav2 navigation nodes
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[('cmd_vel', 'cmd_vel_nav')]  # Use velocity smoother
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # Lifecycle manager for navigation nodes
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': LaunchConfiguration('autostart'),
            'node_names': [
                'controller_server',
                'planner_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor'
            ]
        }]
    )

    # RViz (optional)
    rviz_config = os.path.join(pkg_dir, 'config', 'nav2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        # Arguments
        use_slam_arg,
        map_arg,
        params_arg,
        autostart_arg,
        use_rviz_arg,

        # Robot description
        robot_state_publisher,

        # SLAM or Localization
        slam_launch,
        map_server,
        amcl,
        lifecycle_manager_localization,

        # Navigation stack
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager_navigation,

        # Visualization
        rviz_node,
    ])
