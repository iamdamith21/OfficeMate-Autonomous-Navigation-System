#!/usr/bin/env bash
# location_manager + delivery_manager (the mission FSM).
#
# require_rfid:=false because the MFRC522 sits at 2.65 V on the Mega 3.3 V rail
# and resets whenever its antenna driver powers up, so WAIT_FOR_RFID can never
# complete. TEST ESCAPE: with it false, anyone can open the compartment.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash

ros2 run mission_manager location_manager --ros-args \
    -p map_name:=office_map_v2 \
    -p locations_file:=$HOME/maps/locations.json &
sleep 3
exec ros2 run mission_manager delivery_manager --ros-args \
    -p require_rfid:=false \
    -p file_timeout:=15.0 \
    -p rfid_timeout:=15.0 \
    -p nav_retries:=3 \
    -p base_location:=base_station
