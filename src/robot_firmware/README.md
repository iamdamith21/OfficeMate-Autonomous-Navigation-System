# robot_firmware

Arduino **Mega 2560** firmware for the OfficeMate robot's sensor/actuator hub.
The board drives the motors from `/cmd_vel` and streams every other
sensor/actuator over a single USB-serial link to the Pi. The ROS 2 peer that
parses this link is the [`hardware_bridge`](../hardware_bridge) package; all
odometry fusion (rf2o + this IMU) happens on the Pi — the board never
publishes odometry itself.

```
robot_firmware/
├── arduino/robot_firmware/robot_firmware.ino   # the sketch (v3.0)
├── flash_firmware.sh                           # compile + upload helper
├── CMakeLists.txt / package.xml                # ament_cmake wrapper
└── README.md
```

## Hardware on the board

| Peripheral | Interface | Notes |
|------------|-----------|-------|
| 2× L298N motor drivers | PWM + digital | 4WD skid-steer, one driver per side |
| MPU6050 IMU | I²C `0x68` | gyro yaw-rate feeds the Pi EKF |
| HC-SR04 ultrasonic | TRIG/ECHO | front-low, catches what the lidar misses |
| INA219 power monitor | I²C `0x40` | high-side on the 3S drive battery |
| 20×4 LCD | I²C `0x27` | status line |
| MFRC522 RFID | SPI (3.3 V!) | sender/recipient tag scan |
| IR module | digital | compartment item-present sensor |
| 2× SG90 servos | PWM | split compartment doors |

Full pin map and wiring live in the sketch header and
`robot_description/docs/wiring_diagram_v3.*`.

## Serial protocol (115200 baud, one ASCII line per message)

**Pi → board:** `V,<lin>,<ang>` drive (500 ms watchdog) · `O`/`C` doors
open/close · `L,<text>` LCD line 2.

**Board → Pi:** `S,READY,<ver>` boot · `I,ax,ay,az,gx,gy,gz` IMU 50 Hz ·
`U,<m>` ultrasonic · `B,<V>,<A>` battery · `R,<UID>` RFID · `D,<0|1>` IR ·
`A,DOORS,<state>` door progress · `E,<msg>` diagnostics.

## Flashing

Run on whichever machine the Mega is plugged into (normally the Pi). **Stop
the `arduino_bridge` node first** — the upload needs exclusive access to the
serial port.

```bash
# From an installed workspace:
ros2 run robot_firmware flash_firmware.sh            # default port /dev/arduino
# …or straight from source:
src/robot_firmware/flash_firmware.sh /dev/arduino
```

The first run bootstraps `arduino-cli`, the AVR core and the required
libraries (`Servo`, `LiquidCrystal I2C`, `MFRC522`) into `~/.local`.

### One-time udev rule (stable device name)

```bash
echo 'KERNEL=="ttyACM*",ATTRS{idVendor}=="2341",MODE:="0666",SYMLINK+="arduino"' \
  | sudo tee /etc/udev/rules.d/99-arduino.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```
