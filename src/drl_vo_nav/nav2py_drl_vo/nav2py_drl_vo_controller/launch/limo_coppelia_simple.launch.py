"""
Simple CoppeliaSim + RViz2 Launch File

This launch file starts:
    - Static TF: map -> odom
    - Static TF: base_footprint -> base_link (alias)
    - RViz2

Prerequisites:
    1. Source ROS 2: source /opt/ros/humble/setup.bash
    2. Start CoppeliaSim (after sourcing ROS 2!)
    3. Load and play the limo.ttt scene

Usage:
    Terminal 1 (CoppeliaSim):
        source /opt/ros/humble/setup.bash
        cd ~/Downloads/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/
        ./coppeliaSim.sh
    
    Terminal 2 (Launch):
        source /opt/ros/humble/setup.bash
        ros2 launch nav2py_drl_vo_controller limo_coppelia_simple.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('nav2py_drl_vo_controller')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'limo_nav.rviz')
    
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2'
        ),

        # Static TF: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),

        # Static TF: base_footprint -> base_link (some ROS packages expect base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            parameters=[{'use_sim_time': True}]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}]
        ),
    ])
