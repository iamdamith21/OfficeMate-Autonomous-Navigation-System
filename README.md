# OfficeMate — Autonomous Navigation System

**OfficeMate** is a SLAM-based autonomous file-delivery robot built for an
office / faculty environment. It maps its surroundings, localises within them,
and drives itself from a home base to a sender for RFID-authenticated pickup,
then on to a recipient for drop-off — all on a Raspberry Pi 4 running **ROS 2
Humble** on a custom 4WD chassis.

This repository is the robot's ROS 2 workspace (`ros2_ws/`). It is the single
source of truth for the on-robot software and Arduino firmware.

---

## Highlights

- **2D SLAM mapping** with [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox),
  tuned (Huber-loss pose graph, strict loop closure) for a 270° LiDAR.
- **Sensor-fused odometry** — [rf2o](https://github.com/MAPIRlab/rf2o_laser_odometry)
  scan-matching velocities fused with an MPU6050 gyro through an EKF
  (`robot_localization`), which owns the `odom → base_footprint` transform.
- **Autonomous navigation** on the Nav2 stack (AMCL localisation, planner,
  controller, behaviour trees, optional keep-out costmap filter).
- **Delivery mission state machine** — a `DeliveryMission` action server that
  sequences base → pickup → drop-off → base with RFID auth, servo doors and an
  IR item-present check, publishing `/mission_state` throughout.
- **Custom hardware bridge** — an Arduino Mega streams motors, IMU, ultrasonic,
  battery, RFID, IR and door telemetry over one USB-serial link.
- **Web bridge** — a rosbridge WebSocket endpoint for a companion web app.

## Hardware

| Subsystem | Part |
|-----------|------|
| Compute | Raspberry Pi 4B (Ubuntu + ROS 2 Humble) |
| MCU | Arduino Mega 2560 (see [`robot_firmware`](src/robot_firmware)) |
| Drive | 4WD skid-steer, 4× 100 RPM gear motors, 2× L298N |
| LiDAR | LitraTech **LTME-02A** (Ethernet, LDCP, 270° FoV, 30 Hz) |
| IMU | MPU6050 (I²C) |
| Range | HC-SR04 ultrasonic |
| Power monitor | INA219 (high-side, 3S) |
| Auth / payload | MFRC522 RFID, 2× SG90 door servos, compartment IR |
| UI | 20×4 I²C LCD |

## Repository layout

```
ros2_ws/
└── src/
    ├── robot_description/   URDF/xacro model, meshes, RViz configs, wiring docs
    ├── robot_firmware/      Arduino Mega firmware + flash helper
    ├── hardware_bridge/     ROS↔Arduino serial node + sensor publishers
    ├── robot_bringup/       rsp + rf2o + EKF + LiDAR + bridge, one launch
    ├── robot_mapping/       slam_toolbox mapping launch + params + maps
    ├── robot_navigation/    Nav2 params, AMCL localisation, behaviour trees
    ├── robot_interfaces/    custom msg/srv/action (MissionState, DoorControl, …)
    ├── mission_manager/     delivery mission FSM + task scheduler
    ├── web_bridge/          rosbridge WebSocket endpoint for the web app
    ├── ltme_node/           LTME-02A driver (ROS 2 port of the vendor SDK)
    └── rf2o_laser_odometry/ laser-odometry driver (vendored)
```

## Build

Requires ROS 2 Humble and a configured environment.

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

**Map an environment** (drive it manually while SLAM builds the map):

```bash
ros2 launch robot_mapping mapping.launch.py
# when done:
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

**Autonomous navigation** against a saved map:

```bash
ros2 launch robot_navigation navigation.launch.py map:=~/maps/my_map.yaml
```

**Run a delivery mission** (`simulate:=true` for a dry run without hardware):

```bash
ros2 run mission_manager delivery_manager
```

**Flash the Arduino firmware** — see [`src/robot_firmware`](src/robot_firmware).

### LiDAR network

The LTME-02A is an Ethernet device. Bring `eth0` up on the lidar subnet before
mapping/navigation (static `192.168.10.100/24`, no gateway; lidar at
`192.168.10.160`). Wi-Fi (`wlan0`) stays on the normal LAN for the laptop link.

### Visualising on a laptop

RViz runs off-robot over the network (`ROS_DOMAIN_ID=10`, CycloneDDS pinned to
the Wi-Fi interface). Pre-built configs live in `robot_description/rviz/`.

## Project status

Mapping, odometry, the hardware bridge, the robot model and the delivery FSM
are working and verified on-robot. The **Nav2** and **web_bridge** stacks are
scaffolded — they build and launch, and are being tuned against real maps.

## License

Apache-2.0 for first-party packages. Vendored drivers (`ltme_node`,
`rf2o_laser_odometry`) retain their upstream licenses.
