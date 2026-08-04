#!/usr/bin/env bash
# Full navigation stack: bringup (LTME lidar + rf2o + EKF + arduino_bridge)
# then map_server/AMCL/Nav2.
#
# Usage:  ./start_nav.sh [map_name]      default: faculty_map
#
# The map name is an ARGUMENT rather than hardcoded: it used to be pinned to
# office_map, so mapping a new area and then launching nav silently navigated
# the OLD map while location_manager served coordinates from the new one --
# every goal lands in the wrong place with nothing reporting a mismatch.
#
# arduino_dev MUST be passed: this Mega enumerates as 1a86:55d8 (CH340) and
# the udev rule only matches genuine Arduinos (VID 2341), so /dev/arduino does
# not exist. Rule staged at ~/officemate-arduino.rules (needs sudo).
MAP_NAME="${1:-faculty_map}"
MAP_YAML="$HOME/maps/$MAP_NAME.yaml"

if [[ ! -f "$MAP_YAML" ]]; then
    echo "ERROR: no such map: $MAP_YAML"
    echo "available:"
    ls -1 "$HOME"/maps/*.yaml 2>/dev/null | sed 's|.*/|  |;s|\.yaml$||'
    exit 1
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash

echo "Navigating map: $MAP_YAML"
# USE_IMU=0 in the environment brings the stack up gyro-free (rf2o carries yaw
# on its own). Default stays on: rf2o under-reads pivots by ~29% here, so
# dropping the gyro measurably degrades heading.
USE_IMU="${USE_IMU:-1}"
case "$USE_IMU" in 0|false|no) USE_IMU=false ;; *) USE_IMU=true ;; esac
echo "Fusing gyro yaw: $USE_IMU"

exec ros2 launch robot_navigation navigation.launch.py \
    use_imu:=$USE_IMU \
    map:="$MAP_YAML" \
    arduino_dev:=/dev/ttyACM0
