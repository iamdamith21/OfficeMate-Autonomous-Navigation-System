#!/usr/bin/env bash
# check_web.sh — prove the browser can actually reach the robot.
#
# Checks the socket, the four contract topics and the /deliver action, because
# "Robot Online" in the dashboard only means the websocket opened.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash

echo "=== rosbridge listening on 9090? ==="
ss -ltn 2>/dev/null | grep -q ':9090' && echo "  [ OK ] port 9090 open" \
                                      || echo "  [FAIL] nothing on 9090 -> start_webbridge.sh"

echo "=== browser contract topics ==="
tl=$(timeout 20 ros2 topic list 2>/dev/null)
for t in /battery_level /nav/status /ultrasonic/distance /locker/status; do
    grep -qx "$t" <<<"$tl" && echo "  [ OK ] $t" || echo "  [FAIL] $t missing -> api_adapter not running"
done

echo "=== delivery action ==="
timeout 20 ros2 action list 2>/dev/null | grep -qx '/deliver' \
    && echo "  [ OK ] /deliver" \
    || echo "  [FAIL] /deliver missing -> start_mission.sh"

echo "=== live values (proves data flows, not just that topics exist) ==="
timeout 8  ros2 topic echo /nav/status --once 2>/dev/null | head -2
timeout 8  ros2 topic echo /ultrasonic/distance --once 2>/dev/null | head -2
