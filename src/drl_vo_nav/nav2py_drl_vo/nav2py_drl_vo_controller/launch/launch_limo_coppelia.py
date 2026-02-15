"""
CoppeliaSim Integration Launch File for DRL-VO Navigation

This launch file starts the DRL-VO navigation stack without Gazebo,
expecting CoppeliaSim to run externally with the Limo robot.

Topics expected from CoppeliaSim:
    - /scan (sensor_msgs/LaserScan)
    - /odom (nav_msgs/Odometry)
    - /tf (tf2_msgs/TFMessage)

Topics published to CoppeliaSim:
    - /cmd_vel (geometry_msgs/Twist)

Usage:
    1. Start CoppeliaSim with the limo.ttt scene (after sourcing ROS 2)
    2. ros2 launch nav2py_drl_vo_controller launch_limo_coppelia.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Try to get our package directory, fall back to workspace path
    try:
        pkg_dir = get_package_share_directory('nav2py_drl_vo_controller')
    except:
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Path to params file
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            os.path.dirname(pkg_dir),
            'tb3_drl_vo_nav2_params.yaml'
        )
    )
    
    # Map file (you can change this to your map)
    map_file = LaunchConfiguration(
        'map',
        default=os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml')
    )
    
    # RViz config
    rviz_config_file = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (CoppeliaSim) clock'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_file,
            description='Full path to RViz config file'
        ),

        # Static transform: map -> odom (identity, AMCL will update this)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Nav2 Bringup (without localization, as we use ground truth from CoppeliaSim)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'map': map_file,
            }.items()
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])
