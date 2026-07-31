#!/usr/bin/env bash
# SLAM mapping: bringup (LTME lidar + rf2o + EKF + arduino_bridge) + slam_toolbox.
#
# arduino_dev MUST be passed: this Mega enumerates as 1a86:55d8 (CH340) and the
# udev rule only matches genuine Arduinos (VID 2341), so /dev/arduino does not
# exist. Rule staged at ~/officemate-arduino.rules (needs sudo).
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash
exec ros2 launch robot_mapping mapping.launch.py \
    arduino_dev:=/dev/ttyACM0
