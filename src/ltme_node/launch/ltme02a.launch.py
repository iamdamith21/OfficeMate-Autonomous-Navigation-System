#!/usr/bin/env python3
"""
ltme02a.launch.py — LitraTech LTME-02A driver only (publishes /scan).

The LTME-02A is an Ethernet LiDAR. Its factory-default IP is 192.168.10.160,
so the host interface (Pi eth0) must have an address on 192.168.10.0/24.

frame_id defaults to 'laser' to match the robot URDF's laser link, so the
scan drops straight into the existing TF tree in place of the RPLIDAR.

  ros2 launch ltme_node ltme02a.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_address = LaunchConfiguration('device_address')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('device_address', default_value='192.168.10.160',
                              description='IP[:port] of the LTME-02A (factory default 192.168.10.160)'),
        DeclareLaunchArgument('frame_id', default_value='laser',
                              description='TF frame the LaserScan is published in'),
        Node(
            package='ltme_node',
            executable='ltme_node',
            name='ltme_node',
            output='screen',
            parameters=[{
                'device_model': 'LTME-02A',
                'device_address': device_address,
                'frame_id': frame_id,
            }],
        ),
    ])
