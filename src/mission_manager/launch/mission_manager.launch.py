#!/usr/bin/env python3
"""mission_manager.launch.py — start the delivery mission state machine.

  ros2 launch mission_manager mission_manager.launch.py                 # live
  ros2 launch mission_manager mission_manager.launch.py simulate:=true  # dry-run
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulate = LaunchConfiguration('simulate')
    return LaunchDescription([
        DeclareLaunchArgument('simulate', default_value='false',
                              description='Mock nav/doors/RFID/IR to dry-run the FSM'),
        Node(
            package='mission_manager',
            executable='delivery_manager',
            name='delivery_manager',
            output='screen',
            parameters=[{'simulate': LaunchConfiguration('simulate')}],
        ),
    ])
