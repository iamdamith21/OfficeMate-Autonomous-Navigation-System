#!/usr/bin/env python3
"""localization.launch.py — AMCL localization against a saved map.

  ros2 launch robot_navigation localization.launch.py map:=~/maps/my_map.yaml
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = FindPackageShare('robot_navigation')
    nav2_params = PathJoinSubstitution([nav_share, 'nav2', 'nav2_params.yaml'])
    default_map = os.path.join(os.path.expanduser('~'), 'maps', 'my_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Saved map YAML for localization'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map')},
                         {'use_sim_time': False}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=[nav2_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True,
                          'node_names': ['map_server', 'amcl']}]),
    ])
