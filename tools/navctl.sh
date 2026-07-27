#!/usr/bin/env bash
# navctl.sh — start/stop/status the OfficeMate navigation stack reliably.
#
# Two problems this solves, both of which cost real debugging time:
#
#  1. `pkill -f ros2` from an ssh one-liner matches the ssh command itself and
#     kills the session before it kills the stack. Patterns here are anchored
#     to real executables and the script excludes its own PID.
#
#  2. Backgrounding a launch from ssh with `nohup ... &` is unreliable — the
#     process group gets SIGHUP when ssh exits. setsid with stdin/stdout fully
#     detached is what actually survives.
#
# The ros2 CLI daemon caches an RMW at first use. If it was ever started with a
# different RMW than the stack (fastrtps vs cyclonedds), `ros2 node list`
# silently returns NOTHING while the stack is perfectly healthy. start/status
# therefore always stop the daemon first so it respawns with our RMW.

# NOTE: no `set -u` — ROS 2 setup.bash references unbound vars and would abort.
set -o pipefail

export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclonedds.xml"

NAV_LOG=/tmp/nav.log
WEB_LOG=/tmp/web_bridge.log
MAP="${MAP:-$HOME/maps/office_map.yaml}"

# Match the actual node executables, never a shell wrapper.
PATTERN='robot_state_publisher|joint_state_publisher|wheel_joint_pub|rf2o_laser_odometry_node|ekf_node|arduino_bridge|ltme_node|map_server|amcl|lifecycle_manager|controller_server|planner_server|behavior_server|bt_navigator|velocity_smoother|rosbridge_websocket|api_adapter|ros2 launch'

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

kill_stack() {
  local pids
  pids=$(pgrep -f "$PATTERN" | grep -v "^$$\$" || true)
  if [ -n "$pids" ]; then
    echo "$pids" | xargs -r kill 2>/dev/null
    sleep 4
    pids=$(pgrep -f "$PATTERN" | grep -v "^$$\$" || true)
    [ -n "$pids" ] && echo "$pids" | xargs -r kill -9 2>/dev/null
  fi
  sleep 2
}

case "${1:-status}" in
  start)
    kill_stack
    ros2 daemon stop >/dev/null 2>&1
    rm -f "$NAV_LOG" "$WEB_LOG"
    setsid ros2 launch robot_navigation navigation.launch.py "map:=$MAP" \
      </dev/null >"$NAV_LOG" 2>&1 &
    disown
    echo "nav launching (map=$MAP), log=$NAV_LOG"
    ;;
  web)
    # rosbridge + api_adapter, alongside an already-running nav stack.
    setsid ros2 launch web_bridge web_bridge.launch.py \
      </dev/null >"$WEB_LOG" 2>&1 &
    disown
    echo "web_bridge launching, log=$WEB_LOG"
    ;;
  stop)
    kill_stack
    echo "stopped"
    ;;
  status)
    echo "processes: $(pgrep -fc "$PATTERN" 2>/dev/null || echo 0)"
    echo "nodes:     $(ros2 node list 2>/dev/null | wc -l)"
    echo "port 9090: $(ss -ltn 2>/dev/null | grep -c 9090)"
    ;;
  *)
    echo "usage: navctl.sh {start|web|stop|status}" >&2
    exit 2
    ;;
esac
