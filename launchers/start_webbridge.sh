#!/usr/bin/env bash
# rosbridge (:9090) + api_adapter, for the web app.
#
# api_adapter translates the robot's native topics into the browser contract
# the web app expects; without it the dashboard connects but shows nothing:
#   /battery/state    -> /battery_level        (percent 0-100)
#   /mission_state    -> /nav/status           (display text)
#   /ultrasonic/range -> /ultrasonic/distance  (CENTIMETRES, not metres)
#   /doors/state      -> /locker/status
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash
exec ros2 launch web_bridge web_bridge.launch.py
