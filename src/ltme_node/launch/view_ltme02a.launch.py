#!/usr/bin/env python3
"""
view_ltme02a.launch.py — LTME-02A scan + robot model, for visualization.

Brings up:
  * robot_state_publisher  (my_robot_description URDF -> TF: base_link..laser)
  * joint_state_publisher  (zeros for wheels/casters/laser so TF is complete)
  * ltme_node              (LTME-02A driver -> /scan in the 'laser' frame)

This is a standalone test bringup that does NOT touch the RPLIDAR / sllidar
setup or the rf2o/EKF odometry stack — it only exists to confirm the new
LiDAR works and to see its scan against the robot model in RViz.

Run on the Pi (headless):
  ros2 launch ltme_node view_ltme02a.launch.py

Then on the laptop (same ROS_DOMAIN_ID as the Pi):
  rviz2      # Fixed Frame: base_link, add RobotModel + LaserScan (/scan)

Or launch RViz here too if this host has a display:
  ros2 launch ltme_node view_ltme02a.launch.py rviz:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                   PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device_address = LaunchConfiguration('device_address')
    frame_id = LaunchConfiguration('frame_id')
    use_rviz = LaunchConfiguration('rviz')

    pkg_share = FindPackageShare('my_robot_description')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_file]), value_type=str)
    }

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    lidar_node = Node(
        package='ltme_node',
        executable='ltme_node',
        name='ltme_node',
        output='screen',
        parameters=[{
            'device_model': 'LTME-02A',
            'device_address': device_address,
            'frame_id': frame_id,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('device_address', default_value='192.168.10.160',
                              description='IP[:port] of the LTME-02A'),
        DeclareLaunchArgument('frame_id', default_value='laser',
                              description='TF frame the LaserScan is published in'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Also start RViz on this host (needs a display)'),
        rsp_node,
        jsp_node,
        lidar_node,
        rviz_node,
    ])
