# OfficeMate Wiring Reference — v3 (4WD chassis)

Companion to `wiring_diagram_v3.svg/png`. Matches firmware
`arduino/robot_firmware/robot_firmware.ino` v3.0 exactly — if you move a
wire, change the firmware `#define` with it.

## System overview

- **Drive**: 4× 100 RPM DC gear motors, skid-steer. Left pair on L298N #1,
  right pair on L298N #2. Each motor has its own H-bridge channel.
- **Logic power**: Pi 4B from the 10 Ah power bank; Mega from the Pi's USB
  cable. Motor noise cannot brown-out the computers.
- **Drive power**: 18650 3S pack (11.1 V nom, 3200 mAh) → rocker switch →
  10 A fuse → INA219 (high-side) → VBAT rail → both L298Ns + XL4015 buck.
- **Second 3S pack**: use it as a hot-swap spare, **not** wired in parallel.
  Paralleling li-ion packs at different charge levels dumps large equalising
  currents between them. If you ever want true 3S2P, both packs must be at
  identical voltage (±0.05 V) before joining, ideally through a battery
  management/ideal-diode board.
- **5 V buck rail** (XL4015 set to 5.0 V): both SG90s, 20×4 LCD,
  HC-SR04, IR module. **Never feed servos from the Mega's 5 V pin.**
- **Common ground**: battery B−, buck IN−/OUT−, both L298N GND, Mega GND,
  every sensor/servo GND. The Pi shares ground through the USB cable.

## Arduino Mega pin map

| Mega pin | Connects to | Function |
|---|---|---|
| D9 (PWM) | L298N-L ENA | front-left motor speed |
| D22 / D23 | L298N-L IN1 / IN2 | front-left direction |
| D10 (PWM) | L298N-L ENB | rear-left motor speed |
| D24 / D25 | L298N-L IN3 / IN4 | rear-left direction |
| D5 (PWM) | L298N-R ENA | front-right motor speed |
| D26 / D27 | L298N-R IN1 / IN2 | front-right direction |
| D6 (PWM) | L298N-R ENB | rear-right motor speed |
| D28 / D29 | L298N-R IN3 / IN4 | rear-right direction |
| D11 | left SG90 orange | door servo signal |
| D12 | right SG90 orange | door servo signal |
| D30 / D31 | HC-SR04 TRIG / ECHO | ultrasonic |
| D32 | FC-51 IR OUT | file detect (LOW = present) |
| D49 | MFRC522 RST | RFID reset |
| D53 / D51 / D50 / D52 | MFRC522 SDA(SS) / MOSI / MISO / SCK | RFID SPI |
| D20 (SDA) / D21 (SCL) | MPU6050 + INA219 + LCD backpack | I2C bus |
| 5V pin | MPU6050 VCC, INA219 VCC | I2C sensor power (light loads) |
| 3.3V pin | MFRC522 3.3V | **RFID is 3.3 V only — 5 V kills it** |
| GND ×3 | common ground | |
| USB-B | Pi USB-A | serial link + Mega logic power |

## I2C addresses

| Device | Address | Notes |
|---|---|---|
| MPU6050 | 0x68 | AD0 → GND |
| INA219 | 0x40 | A0, A1 → GND |
| LCD 20×4 backpack | 0x27 | PCF8574; some clones are 0x3F |

## L298N modules

Both modules: **keep the onboard 5 V-regulator jumper ON** (11.1 V input is
fine for it) so each board powers its own logic; leave the 5 V pin
**unconnected** (it is an output in this mode).

| L298N #1 (LEFT) | Connects to |
|---|---|
| +12V (VMS) | VBAT rail (post-INA219) |
| GND | common ground |
| OUT1/OUT2 | front-left motor |
| OUT3/OUT4 | rear-left motor |

L298N #2 (RIGHT) is identical with the front-right / rear-right motors.

**Motor polarity check**: with `V,0.1,0.0` all four wheels must drive
forward. Any wheel spinning backwards → swap that motor's two wires at the
L298N output terminals.

## INA219 (high-side current sense)

```
battery + ──[switch]──[10A fuse]── VIN+ ┌────────┐ VIN− ── VBAT rail
                                        │ INA219 │
                    Mega 5V/GND/D20/D21 ┴────────┘
```
It measures **everything downstream** (motors + buck loads). The Pi/power
bank draw is invisible to it by design.

## Power budget sanity

| Load | Worst case |
|---|---|
| 4 motors (stall, L298N limit) | ~2 A/channel — avoid hard stalls |
| 2× SG90 (moving) | ~0.6 A each, spikes |
| LCD + sonar + IR | <150 mA |
| XL4015 rating | 5 A — ample headroom |

## Assembly cautions

1. **MFRC522 on 3.3 V only** (signals tolerate the Mega's 5 V SPI in
   practice, but power must be 3.3 V).
2. Set the XL4015 to **5.0 V with a multimeter before** connecting servos.
3. Twist each motor pair's wires and keep them away from the I2C/IMU wiring.
4. Mount the MPU6050 rigid, near the chassis center, X axis pointing forward.
5. The FC-51's potentiometer sets detect range — tune it so an empty
   compartment reads clear and a single paper sheet on the floor reads present.
6. Fuse goes as close to the battery + terminal as possible.

## Follow-ups still needed in software (tracked)

- URDF: replace 2-wheel + casters with the 4-wheel chassis once the new
  frame dimensions (track width, wheel radius, body size) are measured.
- `properties.xacro` + firmware `WHEEL_RADIUS` / `WHEEL_SEP` + Nav2
  footprint must all be updated from those same measurements.
