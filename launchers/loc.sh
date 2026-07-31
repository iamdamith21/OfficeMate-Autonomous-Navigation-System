#!/usr/bin/env bash
# loc.sh — thin wrapper over officemate_tools/loc.py with the full DDS env.
#
#   ./loc.sh save base_station    # record where the robot is standing NOW
#   ./loc.sh list
#   ./loc.sh get base_station
#   ./loc.sh delete base_station
#
# Setting only ROS_DOMAIN_ID is not enough: discovery is pinned to CycloneDDS
# with a unicast <Peers> list and multicast off, so a client on the default RMW
# discovers nothing and reports it as "service unavailable" rather than
# "wrong middleware".
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash
exec python3 $HOME/officemate_tools/loc.py "$@"
