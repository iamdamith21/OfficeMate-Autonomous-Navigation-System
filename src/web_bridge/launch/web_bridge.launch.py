#!/usr/bin/env python3
"""web_bridge.launch.py — rosbridge websocket + API adapter for the web app.

Starts rosbridge_server on port 9090 (matching officemate-webapp's ROS
websocket client), plus the api_adapter node that translates the robot's
topics into the vocabulary the web app expects.

Requires rosbridge_suite:
  sudo apt install ros-humble-rosbridge-suite

  ros2 launch web_bridge web_bridge.launch.py

The browser runs on another machine, so rosbridge must listen on all
interfaces, not just loopback. Point the web app at this Pi with
  VITE_ROS_BRIDGE_URL=ws://192.168.1.23:9090
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9090',
                              description='rosbridge websocket port'),
        DeclareLaunchArgument('address', default_value='0.0.0.0',
                              description='bind address; 0.0.0.0 = all '
                                          'interfaces so other machines can '
                                          'connect'),
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='rosbridge_websocket', output='screen',
             parameters=[{
                 'port': LaunchConfiguration('port'),
                 'address': LaunchConfiguration('address'),
                 # The web app reconnects every 5 s on close; without a
                 # keepalive a silent NAT/wifi drop leaves rosbridge holding a
                 # dead socket and the dashboard shows a stale "online".
                 'websocket_ping_interval': 10.0,
                 'websocket_ping_timeout': 30.0,
                 # Sensor topics are best-effort on this robot; let rosbridge
                 # match whatever the publisher offers rather than failing to
                 # subscribe.
                 'default_call_service_timeout': 5.0,
             }]),
        Node(package='web_bridge', executable='api_adapter',
             name='api_adapter', output='screen'),
    ])
