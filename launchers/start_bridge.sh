#!/usr/bin/env bash
# arduino_bridge alone, on the real port.
# NOTE: /dev/arduino does not exist — this Mega enumerates as 1a86:55d8 (CH340)
# and the udev rule only matches genuine Arduinos (VID 2341). Until that rule
# is installed (staged at ~/officemate-arduino.rules, needs sudo), the port
# must be passed explicitly.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash
exec ros2 run hardware_bridge arduino_bridge --ros-args -p port:=/dev/ttyACM0
