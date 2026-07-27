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
                 'default_call_service_timeout': 5.0,
                 # Run service calls and action goals off the main thread.
                 # DeliveryMission is a long-running action — with these left
                 # False (the Humble default) dispatching one blocks
                 # rosbridge's main loop, freezing the telemetry stream for
                 # the whole mission and making the dashboard read "offline"
                 # exactly while the robot is busy. Becomes the default in
                 # Jazzy; rosbridge warns about it on every startup.
                 'call_services_in_new_thread': True,
                 'send_action_goals_in_new_thread': True,
             }]),
        Node(package='web_bridge', executable='api_adapter',
             name='api_adapter', output='screen'),
    ])
