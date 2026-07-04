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

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=10

echo "Saving map to $MAP_DIR/$MAP_NAME ..."
ros2 run nav2_map_server map_saver_cli \
    -f "$MAP_DIR/$MAP_NAME" \
    --ros-args -p save_map_timeout:=5.0

if [[ $? -eq 0 ]]; then
    echo ""
    echo "Map saved:"
    ls -lh "$MAP_DIR/${MAP_NAME}"*
    echo ""
    echo "Start navigation with:"
    echo "  ros2 launch my_robot_description navigation.launch.py map:=$MAP_DIR/$MAP_NAME.yaml"
else
    echo "ERROR: map save failed. Is mapping.launch.py running?"
fi
