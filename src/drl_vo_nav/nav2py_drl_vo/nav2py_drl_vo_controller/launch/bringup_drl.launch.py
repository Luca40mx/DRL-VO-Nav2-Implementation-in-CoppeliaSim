"""
DRL-VO Navigation Bringup Launch File

This launch file starts the full Nav2 stack with the DRL-VO controller plugin.

Prerequisites:
    1. CoppeliaSim running with the Limo robot (Lua script publishing /scan, /odom, TF)
    2. ROS 2 Humble sourced

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition


def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    this_pkg_dir = get_package_share_directory('nav2py_drl_vo_controller')
    
    # Path to our custom params file (installed in share directory)
    default_params_file = os.path.join(this_pkg_dir, 'config', 'drl_nav2_params.yaml')
    
    # Empty map for CoppeliaSim (all free space)
    default_map_file = os.path.join(this_pkg_dir, 'config', 'empty_map.yaml')
    
    # Robot description
    urdf_file = os.path.join(this_pkg_dir, 'config', 'limo_robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    
    return LaunchDescription([
        # ===== Declare Launch Arguments =====
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (CoppeliaSim) clock'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='True',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file for Nav2'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Whether to start RViz'
        ),

        # ===== Static TF: map -> odom =====
        # This provides a fixed transform since we're using ground truth odometry from CoppeliaSim
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # ===== Robot State Publisher =====
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_desc}
            ]
        ),

        # ===== Map Server =====
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {'yaml_filename': map_yaml_file}
            ]
        ),

        # ===== Lifecycle Manager for Map Server =====
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': ['map_server']}
            ]
        ),

        # ===== Nav2 Navigation Stack (without localization) =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
            }.items()
        ),

        # ===== RViz2 =====
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])
