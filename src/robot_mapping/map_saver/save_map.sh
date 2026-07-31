#!/usr/bin/env bash
# save_map.sh — save the current SLAM map to ~/maps/
#
# Usage (while mapping.launch.py is running):
#   ./save_map.sh [map_name]   default: my_map
#
# Output: ~/maps/<name>.yaml  +  ~/maps/<name>.pgm

MAP_NAME="${1:-my_map}"
MAP_DIR="$HOME/maps"

mkdir -p "$MAP_DIR"

# The FULL DDS env, not just the domain id. Discovery here is pinned to
# CycloneDDS with a unicast <Peers> list (multicast is off), so a client that
# only sets ROS_DOMAIN_ID comes up on the default RMW, discovers nothing, and
# map_saver_cli sits until save_map_timeout reporting no map -- which reads as
# "SLAM produced nothing" rather than "wrong middleware".
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

echo "Saving map to $MAP_DIR/$MAP_NAME ..."
ros2 run nav2_map_server map_saver_cli \
    -f "$MAP_DIR/$MAP_NAME" \
    --ros-args -p save_map_timeout:=30.0

if [[ $? -eq 0 ]]; then
    echo ""
    echo "Map saved:"
    ls -lh "$MAP_DIR/${MAP_NAME}"*
    echo ""
    echo "Start navigation with:"
    echo "  ros2 launch robot_navigation navigation.launch.py map:=$MAP_DIR/$MAP_NAME.yaml"
else
    echo "ERROR: map save failed. Is mapping.launch.py running?"
fi
