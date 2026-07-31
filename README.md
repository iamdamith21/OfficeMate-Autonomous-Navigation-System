# OfficeMate — Autonomous Navigation System

**OfficeMate** is a SLAM-based autonomous file-delivery robot for an office /
faculty environment. It maps its surroundings, localises within them, and drives
itself from a home base to a sender to collect a file, then on to a recipient
for RFID-authenticated drop-off — on a Raspberry Pi 4 running **ROS 2 Humble**
on a custom 4WD skid-steer chassis.

This repository is the robot's ROS 2 workspace and the single source of truth
for the on-robot software, launchers, tools and Arduino firmware.

> **New here? Go straight to [Setup](#setup-from-a-blank-pi).** The order of
> those steps matters — the network and DDS configuration in particular will
> silently produce a robot that looks broken if skipped.

---

## Contents

- [How it fits together](#how-it-fits-together)
- [Hardware](#hardware)
- [Repository layout](#repository-layout)
- [Setup from a blank Pi](#setup-from-a-blank-pi)
- [Daily operation](#daily-operation)
- [The full workflow: map → locations → navigate → deliver](#the-full-workflow)
- [Tools reference](#tools-reference)
- [Troubleshooting](#troubleshooting)
- [Project status](#project-status)

---

## How it fits together

```
        LTME-02A LiDAR ──(Ethernet 192.168.10.x)──┐
                                                  │
  Arduino Mega ──(USB serial, ASCII)── hardware_bridge
   motors, IMU, sonar,                            │
   RFID, IR, doors                                ▼
                                    ┌─────────────────────────┐
                                    │  rf2o  →  EKF  →  odom  │
                                    └─────────────────────────┘
                                                  │
                    ┌─────────────────────────────┴───────────────┐
                    ▼                                             ▼
            slam_toolbox (mapping)                    AMCL + Nav2 (navigation)
                    │                                             │
                    └──────────► ~/maps/<name>.yaml ──────────────┘
                                                                  │
                                        location_manager ◄────────┤
                                     (names → poses, JSON)        │
                                                  │               │
                                        delivery_manager (18-state FSM)
                                                  │
                                        web_bridge (rosbridge :9090)
                                                  │
                                            companion web app
```

Two ideas carry a lot of weight:

1. **Odometry is laser-derived.** There are no wheel encoders. `rf2o` estimates
   velocity by matching successive LiDAR scans, and an EKF fuses that with the
   IMU gyro. Anything that upsets scan matching — jerky velocity steps, pivoting
   on the spot — directly degrades odometry and therefore the map.
2. **Destinations are NAMES, not coordinates.** `location_manager` stores named
   poses in `~/maps/locations.json`, keyed by map name. The web app and the
   mission FSM only ever refer to names, so re-surveying a building does not
   invalidate every saved mission.

---

## Hardware

| Subsystem | Part |
|-----------|------|
| Compute | Raspberry Pi 4B (Ubuntu 22.04 + ROS 2 Humble) |
| MCU | Arduino Mega 2560 (see [`src/robot_firmware`](src/robot_firmware)) |
| Drive | 4WD skid-steer, 4× 100 RPM gear motors, 2× L298N |
| LiDAR | LitraTech **LTME-02A** (Ethernet, LDCP, 270° FoV, 30 Hz) |
| IMU | MPU6500 (I²C) |
| Range | HC-SR04 ultrasonic (front, low) |
| Power monitor | INA219 (high-side, 3S) |
| Auth / payload | MFRC522 RFID, 2× SG90 door servos, compartment IR |
| UI | 20×4 I²C LCD |

**Footprint is 44 × 40 cm**, giving a 0.30 m circumscribed radius. That number
matters more than it looks — see [Troubleshooting](#troubleshooting).

---

## Repository layout

```
ros2_ws/
├── launchers/           ← run the robot from here (see Daily operation)
├── tools/               ← diagnostics and operator tools
└── src/
    ├── robot_description/   URDF/xacro model, meshes, RViz configs, wiring docs
    ├── robot_firmware/      Arduino Mega firmware + flash helper
    ├── hardware_bridge/     ROS↔Arduino serial node + sensor publishers
    ├── robot_bringup/       rsp + rf2o + EKF + LiDAR + bridge, one launch
    ├── robot_mapping/       slam_toolbox mapping launch + params + map_saver
    ├── robot_navigation/    Nav2 params, AMCL localisation, behaviour trees
    ├── robot_interfaces/    custom msg/srv/action (MissionState, locations, …)
    ├── mission_manager/     delivery FSM + location_manager
    ├── web_bridge/          rosbridge WebSocket endpoint + API adapter
    ├── ltme_node/           LTME-02A driver (ROS 2 port of the vendor SDK)
    └── rf2o_laser_odometry/ laser-odometry driver (vendored)
```

---

## Setup from a blank Pi

### 1. Base system

Ubuntu 22.04 (64-bit) + [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html),
then:

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
                    ros-humble-rmw-cyclonedds-cpp \
                    ros-humble-slam-toolbox ros-humble-navigation2 \
                    ros-humble-nav2-bringup ros-humble-robot-localization \
                    ros-humble-rosbridge-suite
sudo rosdep init 2>/dev/null; rosdep update
```

`rmw-cyclonedds-cpp` is **required**, not optional — see step 3.

### 2. Clone and build

```bash
git clone https://github.com/iamdamith21/OfficeMate-Autonomous-Navigation-System.git ~/ros2_ws
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` matters: launch files and Python nodes are then live without
a rebuild, which is most of the day-to-day iteration.

### 3. DDS configuration (do not skip)

Discovery is pinned to **CycloneDDS with a unicast peer list and multicast off**,
because the robot has two network interfaces and letting DDS autoselect broke
peer discovery once the Ethernet LiDAR NIC appeared. Create `~/cyclonedds.xml`
on **both the Pi and the laptop**:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <!-- Pin to the Wi-Fi interface. Use the LAPTOP's iface name here
             on the laptop (e.g. wlp3s0) and the Pi's on the Pi. -->
        <NetworkInterface name="wlan0"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <!-- 20 was tight against ~20 stack processes plus tooling and RViz. -->
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="192.168.1.23"/>   <!-- Pi   -->
        <Peer address="192.168.1.10"/>   <!-- laptop: set to yours -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Every launcher in `launchers/` then exports all three of:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

**Setting only `ROS_DOMAIN_ID` is not enough.** A client on the default RMW
discovers nothing and reports it as *"no data"* or *"service unavailable"* —
never as *"wrong middleware"*. That misdiagnosis has cost real hours here.

### 4. LiDAR network

The LTME-02A is an Ethernet device on its own subnet. Give `eth0` a static
address with **no gateway**, so it never becomes the default route:

```bash
sudo nmcli con mod "Wired connection 1" \
  ipv4.addresses 192.168.10.100/24 ipv4.method manual \
  ipv4.gateway "" ipv4.never-default yes
sudo nmcli con up "Wired connection 1"
ping -c3 192.168.10.160   # the lidar
```

Wi-Fi (`wlan0`) stays on the normal LAN for the laptop link.

> The LTME accepts **only one TCP session**. A stale `ltme_node` blocks the next
> launch from ever connecting, and the symptom is a stack with no `/scan` and no
> `map` frame that says nothing about why. Always stop with
> `launchers/navkill.sh`.

### 5. Arduino

Flash the firmware (see [`src/robot_firmware`](src/robot_firmware)), then find
its device node:

```bash
ls -l /dev/ttyACM*
```

**Every launcher must be given `arduino_dev:=/dev/ttyACM0`.** `/dev/arduino`
does not exist on this build: the Mega enumerates as a CH340 (`1a86:55d8`) and
the shipped udev rule only matches genuine Arduino VIDs (`2341`). To fix it
properly:

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d8", SYMLINK+="arduino"' \
  | sudo tee /etc/udev/rules.d/officemate-arduino.rules
sudo udevadm control --reload && sudo udevadm trigger
```

### 6. Laptop (RViz)

Repeat steps 1–3 on the laptop, build only what RViz needs, and use the
matching config:

```bash
colcon build --packages-select robot_description robot_interfaces
./launchers/start_rviz_mapping.sh    # mapping: map + laser + TF
./launchers/start_rviz.sh            # navigation: costmaps + plans
```

Use the **mapping** config while mapping. The navigation config displays
costmaps and plans that do not exist yet, so it looks broken.

---

## Daily operation

Everything is launched from `launchers/`. Each script exports the full DDS
environment and passes `arduino_dev`, so prefer them over raw `ros2 launch`.

| Script | What it starts |
|---|---|
| `start_mapping.sh` | bringup (LiDAR + rf2o + EKF + bridge) + slam_toolbox |
| `start_nav.sh [map]` | bringup + map_server + AMCL + Nav2 (default `office_map_v2`) |
| `start_locations.sh` | `location_manager` alone, for recording coordinates |
| `start_mission.sh` | `location_manager` + `delivery_manager` (the FSM) |
| `start_drive.sh` | smooth teleop for mapping |
| `start_bridge.sh` | `arduino_bridge` alone, for hardware bring-up |
| `navto.sh` | navigate to saved location names |
| `loc.sh` | save / list / get / delete named locations |
| `navkill.sh` | **stop any stack with this** |

**Stop the stack only with `navkill.sh`.** Run as an ssh one-liner,
`pkill -f "ros2 launch"` matches the ssh command itself and kills your own
session (exit 255), leaving the stale `ltme_node` described above.

---

## The full workflow

### 1. Map the environment

```bash
launchers/start_mapping.sh                 # on the Pi
launchers/start_rviz_mapping.sh            # on the laptop
ssh -t <pi> '~/ros2_ws/launchers/start_drive.sh'   # in its own terminal
```

`start_drive.sh` needs a **real TTY** (`ssh -t`), because it reads raw
keypresses. Keys: `w`/`s` drive, `a`/`d` bank, `x` straighten, `space` stop,
`z` quit.

**Drive for map quality, not speed.** Prefer `a`/`d` arcs over `q`/`e` pivots —
pivoting scrubs all four wheels and is the worst case for scan matching. Leave
the speed defaults alone; they sit at the calibrated ceilings, and exceeding
them saturates the firmware turn gain and smears walls. Revisit places you have
already been so slam_toolbox can close loops and square the map up.

Then save it:

```bash
src/robot_mapping/map_saver/save_map.sh office_map_v2
```

### 2. Record named locations

Locations can be recorded **while mapping is still running** —
`location_manager` reads the `map → base_footprint` TF, not `/amcl_pose`, so no
AMCL relocalisation is needed and there is no convergence error in the pose.

```bash
launchers/start_locations.sh               # keep the mapping stack running
# drive to a spot, stop, let it settle, then:
launchers/loc.sh save dean_office
launchers/loc.sh list
```

Heading is stored too, not just x/y — the robot arrives **facing** the way it
faced when saved, which is what makes a drop-off repeatable.

### 3. Check the locations are actually reachable ← easy to skip, do not

```bash
tools/loccost.py
```

**Parking the robot somewhere proves it fits, not that Nav2 will drive there.**
Cells within the robot's 0.30 m circumscribed radius of an obstacle are lethal
regardless of `inflation_radius`. A base station parked against a wall is the
classic failure. `loccost.py` reports the costmap cost at every saved location
and the nearest free cell:

| cost | meaning |
|---|---|
| 0 | free — good goal |
| 1–98 | inflated — the controller may refuse |
| 99 | inscribed — Nav2 will not drive here |
| 100 | lethal |
| −1 | unknown / off-map |

Re-save anything non-zero from about 0.5 m further out.

### 4. Navigate

```bash
launchers/start_nav.sh office_map_v2
tools/globalloc.py ~/maps/office_map_v2.yaml --publish   # seed AMCL
tools/checkloc.py  ~/maps/office_map_v2.yaml             # >70% good
launchers/navto.sh dean_office
launchers/navto.sh dean_office room_1:5 room_2           # sequence, 5 s wait
```

**Always `checkloc.py` before letting Nav2 drive.** AMCL's covariance is
meaningless while stationary (`update_min_d`), so it will happily report
confidence in a completely wrong pose. `navto.sh --snap` nudges a goal onto the
nearest cell with real clearance without modifying your saved locations.

**Quit teleop before navigating.** `start_drive.sh` republishes `/cmd_vel` at
20 Hz *including zeros when idle*, so leaving it running overwrites every Nav2
command ~50 ms later and the robot sits still while the log blames the
controller.

### 5. Run a delivery mission

```bash
launchers/start_mission.sh
tools/deliver.py --sender room_1 --recipient room_2
tools/deliver.py --sender room_1 --recipient room_2 --dry   # no hardware
```

`--dry` / `simulate:=true` needs neither LiDAR nor Nav2 and is the way to
re-check FSM routing after an edit.

---

## Tools reference

| Tool | Purpose |
|---|---|
| `loccost.py` | costmap cost at every saved location — **run after saving** |
| `checkloc.py` | % of scan endpoints landing on mapped walls (>70% good) |
| `globalloc.py` | brute-force global relocalisation, seeds `/initialpose` |
| `pickgoal.py` | list reachable goals sorted by **cost**, not distance |
| `freespace.py` | can the planner reach this cell? |
| `navto.py` | navigate to saved names, with `--snap` and waypoint waits |
| `drive.py` | ramped 20 Hz teleop built for map quality |
| `loc.py` | save / list / get / delete named locations |
| `deliver.py` | run a delivery mission by name |
| `calibrate.py` | measure achieved vs commanded linear velocity |
| `turntest.py` | measure achieved vs commanded turn rate (gyro ground truth) |
| `sensor_check.py` | one-shot read of every sensor on the Mega |
| `navctl.sh` | stack status, queried with `--no-daemon` |

Both `globalloc.py` and `checkloc.py` take the **map YAML as a positional
argument**.

---

## Troubleshooting

These all present as "the robot is dead" or "the hardware is broken", and none
of them logs anything that points at the cause.

**Nothing moves, but Nav2 says it is navigating.** Teleop is still running and
fighting Nav2 for `/cmd_vel`. Check `ros2 topic info /cmd_vel --verbose` for an
`officemate_drive` publisher. The giveaway is `arduino_bridge` logging commands
that alternate with zeros every ~50 ms.

**Goals abort with "collision ahead" / "Controller patience exceeded".** Check
the *goal* first with `loccost.py` — a goal in inflated space produces exactly
this, and nothing names the goal. Only after ruling that out, suspect the sonar
`range_layer` (currently `enabled: false`; the HC-SR04 read 0.37/0.60/0.92/1.44/
1.82/3.0 m while stationary facing a static scene, and with a 69° cone every
short reading painted a lethal arc dead ahead).

**"Failed to make progress" with no collision warning, robot wedged.** If the
LiDAR shows returns closer than 0.30 m, the robot is inside its own
circumscribed radius: it cannot rotate without sweeping the footprint into the
obstacle. `allow_reversing` is now `true` so it has a legal move; otherwise
teleop it into open space.

**`ros2 node list` returns nothing while the stack is running.** The CLI
daemon's discovery cache handles this unicast-peer setup badly. Re-check with
`--no-daemon` — or `navctl.sh status`, which does — before believing the stack
is down. A genuine RMW mismatch gives the identical symptom.

**No `/scan`, no `map` frame, stack otherwise silent.** Stale `ltme_node`
holding the LTME's single TCP session, or the LiDAR has dropped off the network.
Check `cat /sys/class/net/eth0/carrier`: `1` with no ping reply means the LiDAR
has power but its network stack is not running (power-cycle it); `0` means no
electrical link at all (power or cable).

**Map has free-space "spokes" radiating through walls.** No-return beams
reported as `+inf` get ray-traced as free space out to max range. The driver
reports them as `0.0` instead; if this reappears, check that fix in
`ltme_node.cpp`.

**Jerky motion.** `controller_frequency` is 5 Hz because the Pi is CPU-bound —
`rf2o` alone is ~37% and `ltme_node` ~14%, and `ekf_node` reports "Failed to
meet update rate" under load. Raising the controller rate on a saturated CPU
makes commands later, not smoother. Reduce LiDAR/rf2o load first.

---

## Project status

**Working and verified on-robot:** mapping, laser+gyro odometry, the hardware
bridge, the robot model, the 18-state delivery FSM (both timeout branches
verified in simulation), named locations, and Nav2 navigation to a named
location with real clearance.

**Known open items:** the left door servo is destroyed and never driven
(`DOOR_LEFT_PRESENT 0`); the MFRC522 needs a 3.3 V supply that holds under its
antenna load (measured 2.65 V), so `require_rfid:=false` for now — which means
anyone can open the compartment; the sonar `range_layer` is disabled; the
firmware pin map still needs reconciling with the dot board before flashing; and
`rosbridge` has **no authentication**, so the web app link should not be exposed
beyond a trusted network or an SSH tunnel.

## License

Apache-2.0 for first-party packages. Vendored drivers (`ltme_node`,
`rf2o_laser_odometry`) retain their upstream licenses.
