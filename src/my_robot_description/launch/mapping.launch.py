#!/usr/bin/env python3
"""
mapping.launch.py — one-command SLAM mapping.

Starts:
  1. Full robot bringup  (RSP, JSP, rf2o odom+TF, LiDAR, wheel visualisation)
  2. SLAM Toolbox        (async online mapping → publishes map → odom TF)
  3. cmd_vel serial bridge (forwards /cmd_vel to the Arduino motor driver)
     └─ requires: pip/apt package "pyserial" (python3-serial) on the Pi
     └─ Arduino connected via USB → /dev/arduino (udev rule in arduino/ folder)

Run on Pi:
  ros2 launch my_robot_description mapping.launch.py                # RPLidar
  ros2 launch my_robot_description mapping.launch.py lidar:=ltme    # LTME-02A
  ros2 launch my_robot_description mapping.launch.py lidar:=ltme use_arduino:=true

  (lidar:=ltme needs eth0 on the lidar subnet — static 192.168.10.100/24,
   LTME-02A at 192.168.10.160.)

After mapping, save the map:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

RViz on laptop (ROS_DOMAIN_ID=10):
  rviz2 -d ~/ros2_ws/src/my_robot_description/rviz/mapping.rviz
  All displays (RobotModel, TF, LaserScan, Odometry, Map, Pose, SLAM graph)
  are pre-configured — nothing to add manually.
"""
import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_CMD_VEL_BRIDGE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'scripts', 'cmd_vel_serial_bridge.py'
)


def generate_launch_description():
    pkg_share = FindPackageShare('my_robot_description')

    use_arduino = DeclareLaunchArgument(
        'use_arduino', default_value='false',
        description='Start the cmd_vel serial bridge for the Arduino motor driver')

    arduino_dev = DeclareLaunchArgument(
        'arduino_dev', default_value='/dev/arduino',
        description='Arduino serial device (udev symlink)')

    lidar = DeclareLaunchArgument(
        'lidar', default_value='rplidar',
        description="LiDAR to map with: 'rplidar' (default) | 'ltme' (LTME-02A)")

    ltme_address = DeclareLaunchArgument(
        'ltme_address', default_value='192.168.10.160',
        description='LTME-02A IP[:port] (used when lidar:=ltme)')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'bringup.launch.py'])
        ]),
        launch_arguments={
            'lidar': LaunchConfiguration('lidar'),
            'arduino_dev': LaunchConfiguration('arduino_dev'),
            'ltme_address': LaunchConfiguration('ltme_address'),
        }.items(),
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'slam_params.yaml']),
            {'use_sim_time': False},
        ],
    )

    cmd_vel_bridge = ExecuteProcess(
        cmd=['python3', _CMD_VEL_BRIDGE, LaunchConfiguration('arduino_dev')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_arduino')),
    )

    return LaunchDescription([
        use_arduino,
        arduino_dev,
        lidar,
        ltme_address,
        bringup,
        slam_node,
        cmd_vel_bridge,
    ])
