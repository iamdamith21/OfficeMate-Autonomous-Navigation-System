#!/usr/bin/env python3
"""web_bridge.launch.py — rosbridge websocket + API adapter for the web app.

Starts rosbridge_server on port 9090 (matching officemate-webapp's ROS
websocket client), plus the api_adapter node. Requires rosbridge_suite:
  sudo apt install ros-humble-rosbridge-suite

  ros2 launch web_bridge web_bridge.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9090',
                              description='rosbridge websocket port'),
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='rosbridge_websocket', output='screen',
             parameters=[{'port': LaunchConfiguration('port')}]),
        Node(package='web_bridge', executable='api_adapter',
             name='api_adapter', output='screen'),
    ])
