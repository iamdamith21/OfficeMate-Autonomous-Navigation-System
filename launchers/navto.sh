#!/usr/bin/env bash
# navto.sh — navigate to saved location NAMES via Nav2, with the full DDS env.
#
#   ./navto.sh --list
#   ./navto.sh base_station
#   ./navto.sh base_station sender_desk:5 recipient_desk
#
# Setting only ROS_DOMAIN_ID is not enough: discovery is pinned to CycloneDDS
# with a unicast <Peers> list and multicast off, so a client on the default RMW
# discovers nothing and reports it as "action server not available" rather than
# "wrong middleware".
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash
exec python3 $HOME/officemate_tools/navto.py "$@"
