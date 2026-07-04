#!/usr/bin/env python3
"""
display.launch.py — visualise the robot in RViz2.

Nodes launched:
  • robot_state_publisher  — publishes /robot_description and TF tree
  • joint_state_publisher  — broadcasts 0.0 for all non-driven joints
                             (keeps the lidar TF alive without hardware)
  • joint_state_publisher_gui — sliders so you can spin wheels / lidar
  • rviz2                  — 3-D visualisation

Run:
  ros2 launch my_robot_description display.launch.py
  ros2 launch my_robot_description display.launch.py use_rviz:=false
  ros2 launch my_robot_description display.launch.py use_jsp_gui:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('my_robot_description')

    # ── Arguments ──────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock from a simulator instead of wall clock')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2')

    use_jsp_gui_arg = DeclareLaunchArgument(
        'use_jsp_gui', default_value='true',
        description='Launch joint_state_publisher_gui (interactive sliders)')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz     = LaunchConfiguration('use_rviz')
    use_jsp_gui  = LaunchConfiguration('use_jsp_gui')

    # ── Paths ───────────────────────────────────────────────────────────────
    urdf_file   = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'robot_view.rviz'])

    robot_description = {'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)}

    # ── Nodes ───────────────────────────────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # Non-GUI joint_state_publisher: runs when GUI is OFF.
    # Publishes 0.0 for all continuous joints so the TF tree is always complete.
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_jsp_gui),
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_jsp_gui),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        use_jsp_gui_arg,
        rsp_node,
        jsp_node,
        jsp_gui_node,
        rviz_node,
    ])
