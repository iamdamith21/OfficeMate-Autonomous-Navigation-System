# OfficeMate — Supervisor Demo Runbook

Every command, in the order it must be run, with what each `.sh` actually does
and what "good" looks like. Nothing here needs improvising in front of an
audience.

**Two machines.** `PI` = on the robot (`ssh damith-raspberry@192.168.1.23`).
`LAPTOP` = your machine. Every launcher exports the full CycloneDDS environment
itself, so never run a bare `ros2 launch` — see [Why the launchers exist](#why-the-launchers-exist).

**Terminals you will need:** 4 (Pi stack · Pi teleop · laptop RViz · laptop web app).

---

## 0. Pre-flight — do this BEFORE the supervisor is in the room

```bash
# PI
~/fw_testing/preflight.sh
```

It checks, and refuses to pass silently if any of these are wrong:

| Check | Good | If bad |
|---|---|---|
| Motor battery under load | stays **> 10.5 V** | **STOP** — see [Known issues](#known-issues-be-honest-about-these) |
| LiDAR link | `carrier 1` + ping replies | reseat Ethernet / power-cycle lidar |
| Mega firmware | `S,READY,v11-rfid-doors` | reflash, [§8](#8-reflash-the-mega) |
| `/dev/ttyACM0` | present | check USB |
| Stale ROS nodes | none | `~/fw_testing/navkill.sh` |
| Disk + load | load < 2 idle | `kill` whoopsie, clear `/var/crash` |

---

## 1. Mapping a new area

### 1a. Start SLAM (PI, terminal 1)
```bash
~/fw_testing/start_mapping.sh
```
> bringup (LiDAR + rf2o + EKF + arduino_bridge) **+ slam_toolbox**.
> Wait for `/scan` before driving.

### 1b. RViz for mapping (LAPTOP, terminal 3)
```bash
~/ros2_ws/launchers/start_rviz_mapping.sh
```
> Loads `mapping.rviz` — map, laser, TF.
> **Use this one, not `start_rviz.sh`** — the navigation config shows costmaps
> and plans that do not exist yet, so it looks broken.

### 1c. Drive to build the map (PI, terminal 2)
```bash
ssh -t damith-raspberry@192.168.1.23 '~/fw_testing/start_drive.sh'
```
> `-t` is **required** — the teleop reads raw keypresses and needs a real TTY.

| key | action |
|---|---|
| `w` / `s` | forward / reverse |
| `a` / `d` | bank left / right (keeps forward speed → smooth arc) |
| `x` | straighten |
| `space` | stop |
| `z` | quit |

**Drive for map quality, not speed.** Prefer `a`/`d` arcs over `q`/`e` pivots —
pivoting scrubs all four wheels and is the worst case for scan matching, and
there are no wheel encoders here: rf2o's scan matching *is* the odometry.
Revisit places you have already been so slam_toolbox can close loops.

### 1d. Save the map (PI, terminal 2)
```bash
~/ros2_ws/src/robot_mapping/map_saver/save_map.sh office_map_v2
```
> Writes `~/maps/office_map_v2.{yaml,pgm}`.

---

## 2. Recording delivery locations

Do this **while the mapping stack is still running** — `location_manager` reads
the `map → base_footprint` TF, not `/amcl_pose`, so there is no AMCL
convergence error in the pose.

```bash
# PI, terminal 2
~/fw_testing/start_locations.sh          # location_manager alone

# drive to each spot, STOP, let it settle, then:
~/fw_testing/loc.sh save base_station
~/fw_testing/loc.sh save sender_desk
~/fw_testing/loc.sh save recipient_desk
~/fw_testing/loc.sh list
```

Heading is stored too, so the robot arrives **facing** the way it faced when
saved.

### 2a. Check the locations are reachable — DO NOT SKIP
```bash
python3 ~/officemate_tools/loccost.py
```
> Parking the robot somewhere proves it **fits**, not that Nav2 will drive
> there. Cells within the robot's 0.30 m circumscribed radius of a wall are
> lethal whatever `inflation_radius` says.

| cost | meaning |
|---|---|
| 0 | free — good goal |
| 1–98 | inflated — controller may refuse |
| 99 | inscribed — Nav2 will not drive here |
| 100 | lethal |

Anything non-zero:
```bash
python3 ~/officemate_tools/nudge_locations.py            # dry run
python3 ~/officemate_tools/nudge_locations.py --apply    # then RESTART the mission stack
```
> `location_manager` caches locations at startup — restart it or the old poses
> stay live.

---

## 3. Navigation

```bash
# PI terminal 1 — stop mapping first
~/fw_testing/navkill.sh
~/fw_testing/start_nav.sh office_map_v2      # map name is an ARGUMENT

# PI terminal 2 — localise, then PROVE it
python3 ~/officemate_tools/globalloc.py ~/maps/office_map_v2.yaml --publish
python3 ~/officemate_tools/checkloc.py  ~/maps/office_map_v2.yaml
```

`checkloc` verdict: **>70% good, <40% do not drive.** Always run it — AMCL's
covariance is meaningless while stationary (`update_min_d`), so it will happily
report confidence in a completely wrong pose.

```bash
# LAPTOP terminal 3 — navigation RViz (costmaps, plans)
~/ros2_ws/launchers/start_rviz.sh
```

### Drive to a saved location by name
```bash
~/fw_testing/navto.sh recipient_desk
~/fw_testing/navto.sh base_station sender_desk:5 recipient_desk   # sequence, 5 s wait
~/fw_testing/navto.sh --list
~/fw_testing/navto.sh recipient_desk --dry     # resolve only, move nothing
```

> **Quit teleop before navigating.** `start_drive.sh` republishes `/cmd_vel` at
> 20 Hz *including zeros when idle*, so leaving it running overwrites every
> Nav2 command ~50 ms later and the robot sits still while the log blames the
> controller.

---

## 4. The web app

### 4a. rosbridge on the robot (PI, terminal 1 — after nav is up)
```bash
~/fw_testing/start_webbridge.sh
```
> rosbridge on **:9090** + `api_adapter`, which translates the robot's native
> topics into the browser contract.

### 4b. Confirm the link before you rely on it
```bash
~/fw_testing/check_web.sh
```
> Verifies the port is listening, the four browser topics are publishing, and
> the `/deliver` action exists. **Run this rather than trusting the green dot.**

### 4c. Start the web app (LAPTOP, terminal 4)
```bash
cd ~/officemate-webapp && npm run dev
```
> Front end **http://localhost:5173**, API :5000.
> Requires `.env` with `VITE_ROS_BRIDGE_URL=ws://192.168.1.23:9090` — **without
> it the app silently falls back to `localhost:9090`, looks fine, and can never
> reach the robot.**

Sign in as admin. The dashboard should show **Robot Online**, live navigation
status, sonar and battery.

### 4d. Sharing locations with the web app
The robot owns the poses; the app owns the labels. The mapping lives in
`src/constants/index.js` → `NAV_LOCATIONS`:

| App label | Robot name |
|---|---|
| Dean Sir Office | `base_station` |
| Room 1 | `sender_desk` |
| Room 2 | `recipient_desk` |

After re-mapping or re-saving locations, refresh the coordinates:
```bash
~/fw_testing/loc.sh list      # copy x / y / yaw into NAV_LOCATIONS
```
Missions are sent by **name**, so a re-survey does not invalidate stored
deliveries; the coordinates are only so the app can draw them and drive Nav2
directly from the **Send robot to** panel.

---

## 5. Full delivery run (the actual demo)

```bash
# PI terminal 1 — with nav + webbridge already up
~/fw_testing/start_mission.sh
```
> `location_manager` + `delivery_manager` (the 18-state FSM),
> `require_rfid:=true`, 45 s file and RFID timeouts (long enough for a person).

**Dry rehearsal first — needs no lidar and no Nav2:**
```bash
python3 ~/officemate_tools/deliver.py --sender sender_desk --recipient recipient_desk --dry
```
> Preflights localisation, all three names, the action server, IR and doors
> **without sending anything.**

**Then run it from the web app:** create a delivery (Room 1 → Room 2), then
press **🤖 Send Robot** on the admin dashboard.

What the supervisor will see:

| # | State | What happens |
|---|---|---|
| 2 | `NAVIGATE_TO_SENDER` | drives to Room 1 |
| 4–5 | `OPEN_SENDER_DOOR` → `WAIT_FOR_FILE` | doors open, **place the file** (IR detects it) |
| 6–7 | `CLOSE_SENDER_DOOR` → `NAVIGATE_TO_RECIPIENT` | drives to Room 2 |
| 9–10 | `WAIT_FOR_RFID` → `VERIFY_RFID` | **tap the card** (45 s) |
| 11–12 | `OPEN_RECIPIENT_DOOR` → `WAIT_FOR_FILE_REMOVAL` | **take the file out** |
| 14 | `RETURN_TO_BASE` | drives home |

The dashboard banner shows the FSM's own state and can cancel mid-run — cancel
is a real transition: it closes the compartment and routes to `RETURN_TO_BASE`
rather than stopping with the doors open.

**Timeouts are outcomes, not crashes:** no file → return to base; no card →
return the files to the sender.

---

## 6. Debugging commands (have these ready)

```bash
# Health
~/fw_testing/preflight.sh                       # everything at once
~/fw_testing/navctl.sh status                   # node list, queried --no-daemon
uptime                                          # load; >4 on 4 cores is trouble

# Localisation
python3 ~/officemate_tools/checkloc.py ~/maps/office_map_v2.yaml
python3 ~/officemate_tools/globalloc.py ~/maps/office_map_v2.yaml --publish

# Goals
python3 ~/officemate_tools/loccost.py           # are saved locations reachable?
python3 ~/officemate_tools/pickgoal.py          # reachable goals sorted by COST

# Hardware
~/fw_testing/rfid_check.sh 45                   # tag reads + reader health
ros2 topic echo /compartment/occupied --once    # IR
ros2 service call /doors/open  std_srvs/srv/Trigger
ros2 service call /doors/close std_srvs/srv/Trigger
ros2 topic echo /battery/state --once           # VOLTAGE is trustworthy, current is not

# Live data
ros2 topic hz /scan                             # ~30 Hz
ros2 topic hz /imu/data_raw                     # ~45 Hz (measure >10 s)
ros2 topic info /cmd_vel --verbose              # who is publishing?

# Stop everything
~/fw_testing/navkill.sh
```

### The five failures that look like something else

| Symptom | Real cause | Check |
|---|---|---|
| Robot still, Nav2 "navigating" | teleop fighting Nav2 for `/cmd_vel` | `ros2 topic info /cmd_vel --verbose` → look for `officemate_drive` |
| "collision ahead" / patience exceeded | goal in inflated space | `loccost.py` **before** suspecting sensors |
| "Failed to make progress", wedged | inside its own 0.30 m radius — cannot rotate | lidar returns < 0.30 m |
| No `/scan`, no map frame | stale `ltme_node` (LiDAR allows ONE TCP session) or link down | `cat /sys/class/net/eth0/carrier` |
| `ros2 node list` empty | CLI daemon under-reports here | re-check `--no-daemon` |

---

## 7. Shutdown

```bash
~/fw_testing/navkill.sh        # PI — stops nav/mapping/bridge
# Ctrl-C the web app and RViz
```

---

## 8. Reflash the Mega

Only if `preflight.sh` reports the wrong firmware (e.g. after door calibration):
```bash
cd ~/ros2_ws/src/robot_firmware/arduino
~/bin/arduino-cli compile --fqbn arduino:avr:mega robot_firmware
~/bin/arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega robot_firmware
```
> Nothing may hold the port — `fuser -v /dev/ttyACM0` finds the holder; kill it
> **by PID**, never `pkill -f` inside an ssh one-liner (that matches the ssh
> command itself and kills your own session).
>
> Boot should trace `S,BOOT,setup entered → i2c unwedge → imu → rfid → gyro →
> S,GYROCAL → S,READY,v11-rfid-doors`. A missing marker names the hang.

---

## Why the launchers exist

Discovery is pinned to **CycloneDDS with a unicast peer list and multicast
off**. Setting only `ROS_DOMAIN_ID` is not enough — a client on the default RMW
discovers nothing and reports it as *"no data"* or *"service unavailable"*,
never as *"wrong middleware"*. Every launcher exports all three of
`RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID` and `CYCLONEDDS_URI`, and passes
`arduino_dev:=/dev/ttyACM0` (this Mega is a CH340, so `/dev/arduino` does not
exist).

They live in `~/ros2_ws/launchers/` and are version-controlled;
`~/fw_testing/*.sh` are symlinks to them, so the two cannot drift.

---

## Known issues — be honest about these

- **MOTOR POWER (blocking).** The 3S pack reads 11.6 V at rest but collapses to
  **8.22 V** the moment a pivot is commanded, and the wheels achieve
  **0.000 rad/s**. The firmware is correct — it emits symmetric
  `pwmL=-207 pwmR=207`. Until the pack/wiring is fixed the robot will not drive.
- **INA219 current is broken** — reads 0.00 A always. Voltage is trustworthy.
- **Sonar** is a safety stop only, not a costmap layer: parked and facing a
  static scene it reads 0.76–3.00 m (stdev 0.50 m), which would paint phantom
  obstacles.
- **LiDAR Ethernet link has dropped repeatedly** (`carrier 0`). Suspect a
  marginal connector.
- Angular is capped at **0.635 rad/s** by physics: above that both sides are
  already at PWM 255 and nothing changes.
