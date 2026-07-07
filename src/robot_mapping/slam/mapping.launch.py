#!/usr/bin/env python3
"""
mapping.launch.py — one-command SLAM mapping.

Starts:
  1. Full robot bringup  (RSP, JSP, rf2o odom+TF, LTME-02A LiDAR, wheel viz)
  2. SLAM Toolbox        (async online mapping → publishes map → odom TF)

The Arduino bridge (started by bringup) already subscribes to /cmd_vel and
drives the motors, so no separate cmd_vel serial bridge is needed here.

Run on Pi:
  ros2 launch robot_mapping mapping.launch.py

  (needs eth0 on the lidar subnet — static 192.168.10.100/24,
   LTME-02A at 192.168.10.160.)

After mapping, save the map:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

RViz on laptop (ROS_DOMAIN_ID=10):
  rviz2 -d ~/ros2_ws/src/robot_description/rviz/mapping.rviz
  All displays (RobotModel, TF, LaserScan, Odometry, Map, Pose, SLAM graph)
  are pre-configured — nothing to add manually.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('robot_bringup')
    mapping_share = FindPackageShare('robot_mapping')

    arduino_dev = DeclareLaunchArgument(
        'arduino_dev', default_value='/dev/arduino',
        description='Arduino serial device (udev symlink)')

    ltme_address = DeclareLaunchArgument(
        'ltme_address', default_value='192.168.10.160',
        description='LTME-02A IP[:port]')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'bringup.launch.py'])
        ]),
        launch_arguments={
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
            PathJoinSubstitution([mapping_share, 'slam', 'slam_params.yaml']),
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([
        arduino_dev,
        ltme_address,
        bringup,
        slam_node,
    ])
