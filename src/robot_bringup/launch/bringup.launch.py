#!/usr/bin/env python3
"""
bringup.launch.py — real-hardware bringup (Pi only).

Startup sequence:
  t=0s  RSP + JSP + wheel_joint_pub + rf2o + arduino_bridge + LTME-02A LiDAR + rosbridge (port 9090)

LiDAR: LitraTech LTME-02A over Ethernet (LDCP protocol).
  Requires eth0 on the lidar subnet (static 192.168.10.100/24,
  lidar at 192.168.10.160).

Odometry: rf2o estimates velocities and owns the odom->base_footprint TF directly
from scan-matching. Topic: /odom.

WebBridge: rosbridge_server (WebSocket on port 9090) for web dashboard & navigation control.

Launch arguments:
  arduino_dev  : Arduino serial device (default /dev/arduino). The bridge
                 retries every 3 s if the board is unplugged.
  ltme_address : LTME-02A IP[:port] (default 192.168.10.160).

TF tree:
  odom -> base_footprint -> base_link -> body_link -> laser
                                      -> imu_link / ultrasonic_link
                                      -> left_wheel_links / right_wheel_links
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
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
    rosbridge_share = FindPackageShare('rosbridge_server')

    urdf_file = PathJoinSubstitution([desc_share, 'urdf', 'robot.urdf.xacro'])
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_file]), value_type=str)
    }

    # 1. Robot state publisher — broadcasts the URDF TF tree.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # 2. Joint state publisher — merges wheel_joint_states (from wheel_joint_pub)
    #    with default 0.0 values for door joints so RViz can render all links.
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['/wheel_joint_states']}],
    )

    # 3. Wheel joint publisher — integrates odom velocity into left/right wheel angles
    #    and publishes to /wheel_joint_states.
    wheel_pub = ExecuteProcess(
        cmd=['python3', _WHEEL_PUB],
        output='screen',
    )

    # 4. RF2O laser odometry — publishes /odom and owns odom -> base_footprint TF directly
    #    from laser scan matching (zero drift when stationary).
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 7.0,
        }],
        output='screen',
    )

    # 5. Arduino bridge — motors, IMU, ultrasonic, battery, RFID, IR, doors, LCD.
    arduino_bridge = Node(
        package='hardware_bridge',
        executable='arduino_bridge',
        name='arduino_bridge',
        parameters=[{'port': LaunchConfiguration('arduino_dev')}],
        output='screen',
    )

    # 6. LTME-02A LiDAR over Ethernet. Publishes /scan in 'laser' frame.
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

    # 7. Rosbridge WebSocket + rosapi — enables web app dashboard & web navigation (port 9090).
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([rosbridge_share, 'launch', 'rosbridge_websocket_launch.xml'])
        ]),
        launch_arguments={
            'port': '9090',
            'retry_startup_delay': '5.0',
            'send_action_goals_in_new_thread': 'true',
        }.items(),
    )

    # 8. API Adapter — bridges telemetry topics (/battery_level, /nav/status, etc) to Web App.
    _API_ADAPTER = os.path.expanduser('~/officemate_tools/api_adapter.py')
    api_adapter = ExecuteProcess(
        cmd=['python3', _API_ADAPTER],
        output='screen',
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
        arduino_bridge,
        ltme_node,
        rosbridge,
        api_adapter,
    ])
