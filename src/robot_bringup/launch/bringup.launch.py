#!/usr/bin/env python3
"""
bringup.launch.py — real-hardware bringup (Pi only).

Starts, at t=0s:
  RSP + JSP (source_list) + rf2o + EKF + arduino_bridge + wheel_joint_pub
  + the LTME-02A LiDAR driver.

LiDAR: LitraTech LTME-02A over Ethernet (LDCP protocol, respawn=True).
Requires eth0 on the lidar subnet (static 192.168.10.100/24, lidar at
192.168.10.160). The driver has its own connection-retry loop, so no serial
reset dance is needed. It publishes sensor_msgs/LaserScan on /scan in the
'laser' frame.

Odometry: rf2o estimates velocities from the lidar scan; the EKF
(robot_localization, config/ekf.yaml) fuses them with the MPU6050 gyro
from arduino_bridge and owns the odom->base_footprint TF. Fused
output: /odometry/filtered.

Launch arguments:
  arduino_dev : Arduino serial device (default /dev/arduino). The bridge
                retries every 3 s if the board is unplugged.
  ltme_address: LTME-02A IP[:port] (default 192.168.10.160).

TF tree:
  odom -> base_footprint -> base_link -> body_link -> laser
                                      -> imu_link / ultrasonic_link
                                      -> left/right wheel links

Run on Pi:
  ros2 launch robot_bringup bringup.launch.py

On laptop (ROS_DOMAIN_ID=10):
  rviz2 -d ~/ros2_ws/src/robot_description/rviz/bringup.rviz
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_WHEEL_PUB = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'scripts', 'wheel_joint_pub.py'
)


def generate_launch_description():

    desc_share = FindPackageShare('robot_description')
    bringup_share = FindPackageShare('robot_bringup')
    urdf_file = PathJoinSubstitution([desc_share, 'urdf', 'robot.urdf.xacro'])
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_file]), value_type=str)
    }

    # 1. Robot state publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # 2. Joint state publisher — merges wheel_joint_states (from
    #    wheel_joint_pub) with 0.0 defaults for the remaining joints.
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['/wheel_joint_states']}],
    )

    # 3. Wheel joint publisher — integrates rf2o odom velocity into
    #    left/right wheel angles and publishes to /wheel_joint_states.
    wheel_pub = ExecuteProcess(
        cmd=['python3', _WHEEL_PUB],
        output='screen',
    )

    # 4. RF2O laser odometry — velocities only; the EKF owns the odom TF.
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 7.0,
        }],
        output='screen',
    )

    # 5. EKF — fuses rf2o velocities with the MPU6050 gyro; publishes
    #    odom->base_footprint TF and /odometry/filtered.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[PathJoinSubstitution([bringup_share, 'config', 'ekf.yaml'])],
        output='screen',
    )

    # 6. Arduino bridge — motors, IMU, ultrasonic, battery, RFID, IR,
    #    doors, LCD (hardware_bridge package).
    arduino_bridge = Node(
        package='hardware_bridge',
        executable='arduino_bridge',
        name='arduino_bridge',
        parameters=[{'port': LaunchConfiguration('arduino_dev')}],
        output='screen',
    )

    # 7. LiDAR — LitraTech LTME-02A over Ethernet (LDCP). The driver has its
    #    own connection-retry loop. Publishes /scan in the 'laser' frame.
    ltme_node = Node(
        package='ltme_node',
        executable='ltme_node',
        name='ltme_node',
        parameters=[{
            'device_model': 'LTME-02A',
            'device_address': LaunchConfiguration('ltme_address'),
            'frame_id': 'laser',
        }],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
    )

    return LaunchDescription([
        DeclareLaunchArgument('arduino_dev', default_value='/dev/arduino',
                              description='Arduino Mega serial device'),
        DeclareLaunchArgument('ltme_address', default_value='192.168.10.160',
                              description='LTME-02A IP[:port]'),
        rsp_node,
        jsp_node,
        wheel_pub,
        rf2o_node,
        ekf_node,
        arduino_bridge,
        ltme_node,
    ])
