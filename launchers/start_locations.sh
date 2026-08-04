#!/usr/bin/env bash
# location_manager ALONE, for recording named coordinates.
#
# Deliberately not start_mission.sh: that also starts delivery_manager, which
# expects Nav2 up and errors out during mapping. location_manager reads the
# pose from TF (map->base_footprint), so it works straight off slam_toolbox
# while mapping -- no AMCL relocalisation needed.
#
# MAP MUST MATCH THE MAP YOU ARE ACTUALLY SURVEYING.
# location_manager namespaces every saved pose under this name, and it has NO
# idea which map is really loaded -- it just files them wherever it is told.
# This used to be hardcoded to faculty_map, and on 2026-08-02 a whole
# server_room_map survey (server_room_door, research_lab_door, base_station)
# was silently filed under faculty_map, where base_station also OVERWROTE the
# real faculty pose of the same name. sync_locations.py then reported
# 'no locations for map "server_room_map"' with the poses sitting right there
# under the wrong key. Pass MAP explicitly:
#
#     MAP=server_room_map ~/fw_testing/start_locations.sh
#
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash

MAP="${MAP:-server_room_map}"

# Fail loudly rather than filing poses under a map that does not exist.
if [ ! -f "$HOME/maps/${MAP}.yaml" ]; then
    echo "ERROR: no such map: $HOME/maps/${MAP}.yaml" >&2
    echo "available:" >&2
    ls -1 "$HOME"/maps/*.yaml 2>/dev/null | xargs -n1 basename | sed 's/\.yaml$/  /' >&2
    exit 1
fi

echo "location_manager: filing saved poses under map \"${MAP}\""
exec ros2 run mission_manager location_manager --ros-args \
    -p map_name:="${MAP}" \
    -p locations_file:=$HOME/maps/locations.json
