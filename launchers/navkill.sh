#!/usr/bin/env bash
# Stop the whole navigation OR mapping stack.
#
# Lives in a FILE on purpose: running these patterns as an ssh one-liner makes
# pkill match the ssh command itself and kill your own session (exit 255).
#
# ltme_node matters most — the LTME-02A accepts only ONE TCP session, so a
# surviving instance blocks the next launch from ever connecting, and the
# stack comes up with no /scan, no map frame, and nothing that says why.
#
# slam_toolbox matters second. It was MISSING from this list, which made
# switching from mapping to navigation silently broken: a surviving
# slam_toolbox keeps publishing map->odom, so it and AMCL fight over the same
# transform and the robot's pose jumps between two answers.
#
# location_manager is deliberately NOT killed — it only serves saved
# coordinates and is needed by navto.py/deliver.py to resolve names.
# joint_state_publisher and wheel_joint_pub were missing here, so every nav
# launch leaked one of each. Four joint_state_publishers were found alive at
# once, and they quietly inflated every CPU measurement taken while chasing the
# jerky motion -- duplicate publishers also fight over the wheel TF.
for pat in ltme_node slam_toolbox nav2 rf2o arduino_bridge robot_state_pub \
           ekf_node lifecycle_manager map_server amcl controller_server \
           planner_server bt_navigator behavior_server velocity_smoother \
           joint_state_publisher wheel_joint_pub; do
    pkill -9 -f "$pat" 2>/dev/null
done
pkill -9 -f 'launch robot_navigation' 2>/dev/null
pkill -9 -f 'launch robot_mapping' 2>/dev/null
sleep 3
left=$(pgrep -af 'ltme_node|slam_toolbox|nav2|rf2o|arduino_bridge|amcl|map_server|joint_state_publisher|wheel_joint_pub' | wc -l)
echo "remaining: $left"
