#!/usr/bin/env bash
# location_manager ALONE, for recording named coordinates.
#
# Deliberately not start_mission.sh: that also starts delivery_manager, which
# expects Nav2 up and errors out during mapping. location_manager reads the
# pose from TF (map->base_footprint), so it works straight off slam_toolbox
# while mapping -- no AMCL relocalisation needed.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash
exec ros2 run mission_manager location_manager --ros-args \
    -p map_name:=office_map_v2 \
    -p locations_file:=$HOME/maps/locations.json
