#!/usr/bin/env bash
# RViz on the LAPTOP for MAPPING (mapping.rviz: map + laser + TF).
#
# Companion to start_rviz.sh, which loads navigation.rviz -- that config shows
# costmaps and plans, none of which exist while slam_toolbox is building a map.
#
# ROS_DOMAIN_ID and the CycloneDDS config must match the Pi exactly, and
# cyclonedds.xml pins discovery to the wifi interface -- the eth0 lidar NIC
# broke peer discovery when it was left to autoselect.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash
exec rviz2 -d $HOME/ros2_ws/src/robot_description/rviz/mapping.rviz
