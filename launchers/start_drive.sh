#!/usr/bin/env bash
# Smooth teleop for mapping. Run this ON THE PI so /cmd_vel never crosses wifi.
#
# Exports the full DDS env: setting only ROS_DOMAIN_ID is not enough, because
# discovery is pinned to CycloneDDS with a unicast <Peers> list and multicast
# off, so a client on the default RMW discovers nothing and reports it as
# "no data" rather than "wrong middleware".
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash
exec python3 $HOME/officemate_tools/drive.py "$@"
