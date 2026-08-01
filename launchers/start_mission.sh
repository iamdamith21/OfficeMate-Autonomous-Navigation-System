#!/usr/bin/env bash
# location_manager + delivery_manager (the mission FSM).
#
# require_rfid:=true since 2026-08-01: decoupling capacitors on the MFRC522's
# 3.3 V supply fixed it. Before that the rail sagged to 2.65 V whenever the
# antenna driver powered up and the chip reset roughly every other poll, so
# WAIT_FOR_RFID could never complete. Measured after the fix: antenna drops
# 218 -> 0, VersionReg instability 5/20 -> 0/20, and 11 clean reads of a tag.
# Set it back to false only as a test escape -- with it false, anyone can open
# the compartment.
#
# The two 45 s timeouts are deliberately long because a HUMAN has to act inside
# them: place a file in the compartment at the sender, and present a card at the
# recipient. The FSM's 15 s default is fine for scripted runs and far too short
# for a person. Neither timeout is a failure path -- WAIT_FOR_FILE timing out
# returns to base, WAIT_FOR_RFID timing out returns the files to the sender.
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
    -p require_rfid:=true \
    -p file_timeout:=45.0 \
    -p rfid_timeout:=45.0 \
    -p nav_retries:=3 \
    -p base_location:=base_station
