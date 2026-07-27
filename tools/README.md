# tools/

Diagnostic and bring-up scripts for the OfficeMate robot. These previously
lived only in `~/officemate_tools/` and `~/fw_testing/` on the Pi — i.e.
nowhere backed up. They encode measurements and workarounds that were
expensive to rediscover, so they belong in the repo.

Run everything with the robot's environment sourced:

```bash
export ROS_DOMAIN_ID=10 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
```

## Running the stack

| Script | What it does |
|---|---|
| `navctl.sh {start\|web\|stop\|status}` | Start/stop navigation and the web bridge. |

Use this rather than backgrounding `ros2 launch` from ssh by hand. It solves
three things that each cost real debugging time:

* `pkill -f ros2` in an ssh one-liner **matches the ssh command itself** and
  kills your session instead of the stack.
* `nohup ... &` over ssh does not reliably survive; the process group is
  SIGHUP'd on disconnect. `setsid` with fully detached stdio does.
* The `ros2` CLI daemon caches whichever RMW it first saw. If it was ever
  started with `rmw_fastrtps_cpp` while the stack runs CycloneDDS,
  `ros2 node list` returns **nothing at all** while the stack is perfectly
  healthy — a very convincing false alarm. `navctl.sh` stops the daemon so it
  respawns with the right RMW.

## Before you let Nav2 drive

| Script | What it does |
|---|---|
| `checkloc.py` | Projects `/scan` into the map, reports % of endpoints landing on mapped walls. **>70% good, <40% do not navigate.** |
| `globalloc.py <map.yaml> --publish` | Brute-force global localisation; recovers a lost AMCL in ~30 s and seeds `/initialpose`. |
| `freespace.py` | Reports what the planner can *actually* reach, and suggests valid goals. |

Always run `checkloc.py` first. AMCL's covariance is meaningless while the
robot is stationary (`update_min_d`), so it will happily report high confidence
in a completely wrong pose.

`freespace.py` exists because "failed to create plan, no valid path found"
almost always means the goal sits in inflated space, not that the planner is
broken. Pick goals from its output instead of guessing from the map's bounding
box. Note it reads `/global_costmap/costmap`, which is a `nav_msgs/OccupancyGrid`
on a **0–100** scale with −1 unknown — not the raw 0–255 cost range. Using the
raw thresholds reports "0 lethal cells" on a costmap full of walls.

## Measurement

| Script | What it does |
|---|---|
| `sensor_check.py [secs]` | Reads the Mega's serial stream directly (no ROS) and summarises IMU/sonar rates, means and noise. |
| `turntest.py [rates...]` | Commands a sweep of angular velocities through `/cmd_vel_nav` and measures what is achieved, using the **gyro** as ground truth. |
| `calibrate.py` | Linear velocity calibration against `/odometry/filtered`. |
| `navgoal.py <x> <y> [yaw] [timeout]` | Sends a real `NavigateToPose` goal and reports the outcome. |

`turntest.py` is what sized `TURN_GAIN_PIVOT`. The number to watch is not the
peak rate but whether the response is **proportional**: an angular limit set at
the robot's saturation point makes every turn command produce full deflection,
which reads as both slow and jerky.

`navgoal.py` takes the robot pose from **TF (`map` → `base_footprint`)**, not
from `/odometry/filtered` — that topic is in the `odom` frame, so comparing it
against a `map`-frame goal silently reports a meaningless distance.
