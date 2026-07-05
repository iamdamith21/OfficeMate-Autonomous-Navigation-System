#!/usr/bin/env python3
"""
navigation.launch.py — autonomous navigation on a saved map.

Arguments:
  map               : path to saved map YAML             (default ~/maps/my_map.yaml)
  arduino_dev       : Arduino serial device              (default /dev/arduino)
  use_keepout_filter: start costmap keepout filter       (default false)
  keepout_mask      : path to keepout mask YAML          (default maps/keepout_mask.yaml)

Pipeline:
  Robot bringup (incl. arduino_bridge + EKF) → AMCL → Nav2 planner + controller
  cmd_vel flow: controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel
                → arduino_bridge (started by bringup) → motors

Run on Pi:
  ros2 launch robot_navigation navigation.launch.py \
      map:=~/maps/my_map.yaml [use_keepout_filter:=true]

RViz on laptop (ROS_DOMAIN_ID=10):
  rviz2 -d ~/ros2_ws/src/robot_description/rviz/mapping.rviz
  All displays (RobotModel, TF, LaserScan, Odometry, Map) are pre-configured.
  Use "2D Pose Estimate" to init AMCL, then "2D Goal Pose" to navigate.
  (The Pose/SLAM-graph displays stay empty here — those are slam_toolbox-only
  topics, harmless to leave enabled.)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg            = FindPackageShare("robot_navigation")
    nav2_params    = PathJoinSubstitution([pkg, "nav2", "nav2_params.yaml"])
    cf_params      = PathJoinSubstitution([pkg, "nav2", "costmap_filter_params.yaml"])
    bt_xml         = PathJoinSubstitution([pkg, 'behavior_trees',
                                           'navigate_w_replanning_and_recovery.xml'])
    default_map    = os.path.join(os.path.expanduser('~'), 'maps', 'my_map.yaml')
    default_mask   = PathJoinSubstitution([pkg, 'maps', 'keepout_mask.yaml'])

    # ── Launch arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('map',    default_value=default_map,
            description='Full path to saved map yaml'),
        DeclareLaunchArgument('arduino_dev', default_value='/dev/arduino',
            description='Arduino serial device (passed through to bringup)'),
        DeclareLaunchArgument('use_keepout_filter', default_value='false',
            description='Enable keepout zone costmap filter'),
        DeclareLaunchArgument('keepout_mask', default_value=default_mask,
            description='Path to keepout filter mask yaml'),
    ]

    # ── Robot hardware bringup ────────────────────────────────────────────────
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, 'launch', 'bringup.launch.py'])
        ]),
        launch_arguments={
            'arduino_dev': LaunchConfiguration('arduino_dev'),
        }.items(),
    )

    # ── Localisation: map_server + AMCL ──────────────────────────────────────
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[nav2_params,
                    {'yaml_filename': LaunchConfiguration('map')}],
    )

    amcl = Node(
        package='nav2_amcl', executable='amcl',
        name='amcl', output='screen',
        parameters=[nav2_params],
    )

    lc_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['map_server', 'amcl']}],
    )

    # ── Navigation stack ──────────────────────────────────────────────────────
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')],   # output to velocity smoother
    )

    # smoother_server: NOT launched — the custom BT (navigate_w_replanning_
    # and_recovery.xml) never calls a Smooth action (SmacPlanner2D already
    # smooths internally via its own smoother: block), so this was a whole
    # extra process (~12% CPU) doing nothing. Pi 4B can't spare it.

    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[nav2_params,
                    {'default_nav_to_pose_bt_xml': bt_xml}],
    )

    # waypoint_follower: NOT launched — single-goal navigate_to_pose only,
    # no multi-waypoint FollowWaypoints calls anywhere in this project.

    # Velocity smoother:  /cmd_vel_nav (controller) → /cmd_vel (robot)
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=[nav2_params],
        remappings=[
            ('cmd_vel',         'cmd_vel_nav'),   # read controller output
            ('cmd_vel_smoothed', 'cmd_vel'),       # write smoothed command
        ],
    )

    lc_navigation = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'autostart': True,
                     'node_names': [
                         'controller_server', 'planner_server',
                         'behavior_server',   'bt_navigator',
                         'velocity_smoother']}],
    )

    # ── Keepout costmap filter (optional) ─────────────────────────────────────
    # filter_mask_server serves the keepout PGM as OccupancyGrid.
    # costmap_filter_info_server reads mask → publishes /costmap_filter_info.
    # KeepoutFilter in global_costmap reads /costmap_filter_info.
    filter_mask_server = Node(
        package='nav2_map_server', executable='map_server',
        name='filter_mask_server', output='screen',
        parameters=[cf_params,
                    {'yaml_filename': LaunchConfiguration('keepout_mask')}],
        condition=IfCondition(LaunchConfiguration('use_keepout_filter')),
    )

    costmap_filter_info = Node(
        package='nav2_map_server', executable='costmap_filter_info_server',
        name='costmap_filter_info_server', output='screen',
        parameters=[cf_params],
        condition=IfCondition(LaunchConfiguration('use_keepout_filter')),
    )

    lc_filter = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters', output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['filter_mask_server',
                                     'costmap_filter_info_server']}],
        condition=IfCondition(LaunchConfiguration('use_keepout_filter')),
    )

    return LaunchDescription([
        *args,
        bringup,
        # localisation
        map_server, amcl, lc_localization,
        # navigation
        controller_server, planner_server,
        behavior_server, bt_navigator,
        velocity_smoother, lc_navigation,
        # optional: keepout filter
        filter_mask_server, costmap_filter_info, lc_filter,
    ])
