#!/usr/bin/env python3
"""
bringup.launch.py — real-hardware bringup (Pi only).

Startup sequence:
  t=0s  RSP + JSP (source_list) + rf2o + EKF + arduino_bridge
        + wheel_joint_pub start
  lidar:=rplidar (default):
    t=2s  STOP+RESET sent to /dev/sllidar (flush stale serial data)
    t=9s  sllidar_node starts (respawn=True for resilience)
  lidar:=ltme:
    t=0s  ltme_node starts (LitraTech LTME-02A over Ethernet; respawn=True).
          Requires eth0 on the lidar subnet (static 192.168.10.100/24,
          lidar at 192.168.10.160). No serial reset dance needed.

Both LiDARs publish sensor_msgs/LaserScan on /scan in the 'laser' frame, so
rf2o odometry and slam_toolbox are agnostic to which one is used.

Odometry: rf2o estimates velocities from the lidar scan; the EKF
(robot_localization, config/ekf.yaml) fuses them with the MPU9250 gyro
from arduino_bridge and owns the odom->base_footprint TF. Fused
output: /odometry/filtered.

Launch arguments:
  lidar       : which LiDAR to bring up — 'rplidar' (default) | 'ltme'.
  arduino_dev : Arduino serial device (default /dev/arduino). The bridge
                retries every 3 s if the board is unplugged.
  ltme_address: LTME-02A IP[:port] (default 192.168.10.160).

TF tree:
  odom -> base_footprint -> base_link -> body_link -> laser
                                      -> imu_link / ultrasonic_link
                                      -> left_wheel_link
                                      -> right_wheel_link
                                      -> (casters)

Note on laser_joint: sllidar_ros2 de-rotates scan data internally so
the laser frame must stay at 0 rad — spinning it would misalign the
scan in RViz.

Run on Pi:
  ros2 launch robot_bringup bringup.launch.py

On laptop (ROS_DOMAIN_ID=10):
  rviz2 -d ~/ros2_ws/src/robot_description/rviz/bringup.rviz
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_RESET_CMD = (
    'import os,time,termios;'
    'fd=os.open("/dev/sllidar",os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK);'
    'termios.tcflush(fd,termios.TCIOFLUSH);'
    'os.write(fd,bytes([0xa5,0x25]));time.sleep(0.05);'
    'os.write(fd,bytes([0xa5,0x40]));time.sleep(2.0);'
    'termios.tcflush(fd,termios.TCIFLUSH);'
    'os.close(fd);time.sleep(0.3);'
    'print("LiDAR reset OK")'
)

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
    #    wheel_joint_pub) with 0.0 defaults for casters and laser.
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

    # 5. EKF — fuses rf2o velocities with the MPU9250 gyro; publishes
    #    odom->base_footprint TF and /odometry/filtered.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[PathJoinSubstitution([bringup_share, 'config', 'ekf.yaml'])],
        output='screen',
    )

    # 6. Arduino bridge — motors, IMU, ultrasonic, battery, RFID, IR,
    #    doors, LCD (robot_interface package).
    arduino_bridge = Node(
        package='hardware_bridge',
        executable='arduino_bridge',
        name='arduino_bridge',
        parameters=[{'port': LaunchConfiguration('arduino_dev')}],
        output='screen',
    )

    # LiDAR selection: 'rplidar' (default) or 'ltme'.
    lidar = LaunchConfiguration('lidar')
    is_rplidar = IfCondition(PythonExpression(["'", lidar, "' == 'rplidar'"]))
    is_ltme = IfCondition(PythonExpression(["'", lidar, "' == 'ltme'"]))

    # --- RPLIDAR path -------------------------------------------------------
    # 7a. t=2s: flush stale RPLIDAR serial buffer from any previous unclean shutdown.
    reset_lidar = ExecuteProcess(
        cmd=['python3', '-c', _RESET_CMD],
        output='screen',
    )

    # 7b. t=9s: start sllidar. respawn handles rare single-timeout.
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/sllidar',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
    )

    # --- LTME-02A path ------------------------------------------------------
    # LitraTech LTME-02A over Ethernet (LDCP). No serial reset dance; the
    # driver has its own connection-retry loop. Publishes /scan in 'laser'.
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
        condition=is_ltme,
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='rplidar',
                              description="LiDAR to bring up: 'rplidar' | 'ltme'"),
        DeclareLaunchArgument('arduino_dev', default_value='/dev/arduino',
                              description='Arduino Mega serial device'),
        DeclareLaunchArgument('ltme_address', default_value='192.168.10.160',
                              description='LTME-02A IP[:port] (used when lidar:=ltme)'),
        rsp_node,
        jsp_node,
        wheel_pub,
        rf2o_node,
        ekf_node,
        arduino_bridge,
        # RPLIDAR: reset then start sllidar (gated on lidar:=rplidar).
        TimerAction(period=2.0, actions=[reset_lidar], condition=is_rplidar),
        TimerAction(period=9.0, actions=[lidar_node], condition=is_rplidar),
        # LTME-02A: start immediately (gated on lidar:=ltme).
        ltme_node,
    ])
