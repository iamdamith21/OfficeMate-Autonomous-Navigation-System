#!/usr/bin/env python3
"""
rsp.launch.py — robot_state_publisher only (real-hardware / bringup use).

Use this launch file when running on the real robot.
Wheel joint states should come from your motor driver / diff_drive_controller.
The joint_state_publisher here only fills in joints with no hardware source
(caster swivels, lidar rotation) so the TF tree is always complete.

Run:
  ros2 launch my_robot_description rsp.launch.py
  ros2 launch my_robot_description rsp.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('my_robot_description')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock from a simulator')

    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # Publishes 0.0 for joints not driven by hardware (casters, lidar).
    # When diff_drive_controller is running it publishes wheel joint states
    # which override the 0 values for left/right wheels.
    # Set source_list if you need to merge additional joint state topics:
    #   parameters=[{'source_list': ['/joint_states_hw']}]
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        rsp_node,
        jsp_node,
    ])
