/*
 * robot_firmware.ino v11 — OfficeMate sensor/actuator hub — FULL PERIPHERAL SET
 *
 * Board : Arduino Mega 2560
 * Libs  : Wire, SPI, MFRC522, Servo
 *
 * Fitted: the two L298N motor drivers, an MPU6500 IMU, an INA219 power monitor
 * and a 20x4 LCD (all on I2C), an HC-SR04 ultrasonic on TRIG=D40 / ECHO=D41,
 * an MFRC522 RFID reader on hardware SPI, an IR compartment sensor on D43, and
 * two SG90 door servos on D44/D45. Every publisher and command the ROS-side
 * hardware_bridge/arduino_bridge.py knows about is now backed by real hardware.
 *
 * ─── v11: RFID + IR + door servos ─────────────────────────────────────────
 * These three complete the delivery mission FSM in mission_manager: it parks
 * at the dropoff, waits for a tag on /rfid/tag (VERIFY_RFID), calls
 * /doors/open (OPEN_DOOR), waits for /compartment/occupied to go false
 * (WAIT_PACKAGE_REMOVAL), then /doors/close. The serial protocol was already
 * specified and parsed by the bridge — R, D and A,DOORS lines simply had
 * nothing producing them until now.
 *
 * The same no-blocking rule that shaped the sonar and LCD drivers applies, and
 * the RFID reader is the worst offender on the board:
 *
 *   MFRC522 : the library busy-waits for the reader's internal timer on every
 *             card poll, and PCD_Init leaves that timer at 25 ms. With no card
 *             present that is a 25 ms stall — four whole software-PWM periods.
 *             TWO defences, because one is not enough. (1) TReloadReg is cut to
 *             ~2 ms right after PCD_Init (RFID_TIMER_TICKS), which is still an
 *             order of magnitude more air time than a REQA/ATQA exchange needs.
 *             (2) rfidTick returns immediately unless all four wheels are
 *             stopped. Polling while driving buys nothing — the tag is scanned
 *             with the robot parked at the dropoff — so the remaining ~2 ms
 *             stall only ever lands when there is no PWM to disturb.
 *
 *   Servos  : the Servo library is interrupt-driven, so writeMicroseconds() is
 *             a store, not a wait. Door motion is a time-interpolated state
 *             machine (see doorTick) rather than the usual step-and-delay loop.
 *
 * SERVO SAFETY RULE, PAID FOR ONCE ALREADY — never command a servo position
 * that was not asked for, and never step one. A first cut of v11 parked the
 * doors in setup() with a bare doorApply(). Since every flash and every reset
 * re-runs setup(), an arm left open was driven the full sweep into the door
 * frame at ~600 deg/s and stalled there; that broke the left servo, and the
 * stalled SG90's ~700 mA pulled the 5 V rail down until the board would no
 * longer boot. Hence: setup() does not touch the servos, they are attached
 * only for the duration of a commanded move (doorAttach), the first pulse
 * after attaching re-states the CURRENT position so nothing jumps, and they
 * are released once the endpoint settles.
 *
 *   IR      : one digitalRead per 20 ms poll. Free.
 *
 * DOOR MOTION — why it is interpolated on time rather than stepped.
 * The bench calibration sketch moved 2 deg every 60 ms. An SG90 slews at about
 * 600 deg/s, so each step was a ~3 ms jab followed by a 57 ms dead stop: 17
 * discrete kicks a second, which through a long printed arm reads as a violent
 * judder. doorTick instead refreshes once per 20 ms servo frame (finer is
 * wasted — the servo latches one pulse per frame), commands in microseconds so
 * the steps are sub-degree, and eases the travel with a smoothstep so the arm
 * cannot bounce against the door frame at either end.
 *
 * DOOR ANGLES are the bench-calibrated pair and are NOT mirrored — the two
 * arms have different geometry on the frame, so each side keeps its own
 * independent closed/open pair. Angle -> pulse uses the Servo library's own
 * 544-2400 us mapping so these numbers mean exactly what they meant on the
 * bench; SERVO_US_MIN/MAX then clamp the result as a backstop against a
 * mis-edit driving a servo into its internal end stop and grinding.
 *
 * ─── v10.1: INA219 + LCD ──────────────────────────────────────────────────
 * The LCD is driven by a purpose-written incremental driver rather than
 * LiquidCrystal_I2C. That library writes a whole line per blocking call —
 * several milliseconds, most of a software-PWM period — and the motors would
 * stutter on every refresh. Here a shadow buffer is diffed against the target
 * and exactly ONE changed character is pushed per 3 ms tick, so the longest
 * blocking I2C burst is a single character (~135 us). A full 80-cell repaint
 * takes ~240 ms, and a typical update (a few digits) far less.
 *
 * The serial protocol is unchanged from v3.0, so hardware_bridge/arduino_bridge
 * parses `I,` and `U,` lines with no ROS-side edits — those handlers were
 * already written and simply had nothing to consume.
 *
 * ─── v10 CHANGE: IMU + ultrasonic, without disturbing the motors ───────────
 * The hard constraint here is spwmTick() (see "Software PWM" below): the
 * direction pins are chopped in software, so anything that blocks the main
 * loop for more than a fraction of the 6 ms PWM period shows up as visible
 * motor stutter. Both new sensors are therefore written to never block:
 *
 *   HC-SR04 : a 4-state non-blocking machine. The textbook pulseIn() call
 *             busy-waits up to ~30 ms for the echo — five whole PWM periods,
 *             which would wreck the motors. Instead the echo pin is POLLED
 *             from loop() and edges are timestamped with micros(). loop()
 *             iterates far faster than the ~58 us/cm echo resolution, so the
 *             measurement is just as good and nothing ever stalls.
 *
 *   MPU     : one 14-byte burst read at 400 kHz I2C ≈ 350 us, at 50 Hz.
 *             That is ~1.7% duty and the only blocking call left. Retries
 *             (see mpuReadRaw) cost a second transaction on most cycles, so
 *             the measured publish rate is ~40 Hz, not the nominal 50.
 *
 *   Serial  : IMU lines are only emitted when the UART TX buffer has room
 *             (availableForWrite), so a slow reader on the Pi can never make
 *             Serial.print() block the PWM loop. Dropping an IMU sample is
 *             always better than glitching the motors.
 *
 * The gyro is bias-calibrated at boot (ROBOT MUST BE STATIONARY) and can be
 * re-zeroed at any time with the `G` command. This matters: the EKF fuses
 * gyro yaw-RATE, so an uncalibrated 1-2 deg/s bias integrates straight into
 * a steadily rotating odometry estimate.
 *
 * ─── v6 CHANGE: stiction kick + higher floor ──────────────────────────────
 * v5 published fine at 20 Hz but the robot still would not move on slow
 * commands. Cause was mechanical, not electrical: 0.22 m/s maps to PWM 82,
 * which cannot break static friction with four wheels loaded, while 0.5 m/s
 * (PWM 187) drove fine. MIN_PWM is now 90, and a standing start or direction
 * reversal gets KICK_MS at KICK_PWM before falling back to the commanded duty.
 * That keeps genuinely slow mapping speeds usable.
 *
 * ─── v5 CHANGE: software PWM on the DIRECTION pins ────────────────────────
 * The L298Ns on this rig have no working 5 V logic supply — they scavenge
 * power through their logic pins. Consequence: they only stay alive while
 * those pins are high a good fraction of the time.
 *
 * v4 hardware-PWMed the ENABLE lines, which meant low speed = low enable duty
 * = the driver browning out. Measured: the wheels stop responding somewhere
 * around PWM 100, so /cmd_vel at 0.2 m/s (PWM 74) did nothing at all while the
 * 150-255 bench tests worked fine.
 *
 * v5 holds all four enables at solid DC HIGH — maximum scavenged power at any
 * speed — and chops the direction pins in software for speed control. The two
 * concerns are now independent.
 *
 * D30-D37 are not hardware-PWM pins, hence software PWM: ~167 Hz, far above
 * anything the gearboxes respond to. It is cheap because a pin is only written
 * when its state actually changes.
 *
 * THIS IS STILL A WORKAROUND. Give the L298Ns a real 5 V supply (refit the
 * 3-pin 78M05 regulator jumper, or feed 5 V from the buck) and the whole
 * scavenging problem disappears.
 *
 * ─── DAMAGED PINS ON THIS BOARD — DO NOT USE ──────────────────────────────
 *   Measured by pin_health.ino, not assumed:
 *     D2   stuck low  (NO_SOURCE, pull-up cannot lift it)
 *     D5   stuck high (NO_SINK)
 *     D9   stuck low  <- was an enable, moved to D7
 *     D10  stuck low  <- was an enable, moved to D8
 *   D22-D25 and D28-D29 leak to ground (pass pull-up, fail charge retention)
 *   and are avoided. D3 measured healthy despite earlier reports. D0/D1 are
 *   RX0/TX0 and are proven good by uploads working.
 *
 * ─── Wiring — drivers split by SIDE (left / right) ────────────────────────
 *  L298N LEFT  : ENA=7  (front-left)  IN1=30 IN2=31
 *                ENB=8  (rear-left)   IN3=32 IN4=33
 *  L298N RIGHT : ENA=11 (front-right) IN1=34 IN2=35
 *                ENB=12 (rear-right)  IN3=36 IN4=37
 *
 *  Enables are plain DC outputs now, so they no longer need to be PWM pins.
 *  Remove the ENA/ENB jumper caps on both boards. Both L298N GND pins MUST
 *  tie to Mega GND.
 *
 *  NOTE: with a per-side split a dead driver takes out an entire side and the
 *  robot will spin rather than limp.
 *
 * ─── Wiring — sensors (v10) ───────────────────────────────────────────────
 *  MPU6050 / MPU9250 : VCC=5V (breakouts have a 3.3 V LDO), GND, SDA=D20,
 *                      SCL=D21. AD0 low = 0x68, high = 0x69 — both probed.
 *                      D20/D21 read NO_SINK on pin_health: that is just the
 *                      I2C pull-ups, not damage.
 *  HC-SR04           : VCC=5V, GND, TRIG=D40, ECHO=D41. Both measured healthy
 *                      and clear of the damaged-pin list. The Mega is a 5 V
 *                      part so the echo line needs no divider.
 *
 *  IMU MOUNTING: the sketch assumes the breakout lies flat with its +X
 *  silkscreen arrow pointing FORWARD and +Z up. If yours is rotated, fix it
 *  with IMU_AXIS_* below rather than in ROS — the EKF wants a genuine
 *  base_link-aligned reading. Sanity check: rotate the robot counter-clockwise
 *  (left) and gz must go POSITIVE (REP-103 right-hand rule).
 *
 * ─── Serial protocol (115200 baud, one ASCII line per message) ─────────────
 *  Pi → board:
 *    V,<lin m/s>,<ang rad/s>   drive command (watchdog: stop after 500 ms)
 *    M,<left>,<right>          RAW per-side PWM, -255..255
 *    W,<fl>,<fr>,<rl>,<rr>     RAW per-wheel PWM
 *    X                         self-demo: cycles every wheel unaided, ~17 s
 *    Y,<0-3>                   drive only wheel N at full (now same as W)
 *    S                         stop immediately
 *    P                         ping → replies "S,PONG"
 *    G                         re-zero the gyro bias (robot must be still)
 *    Z                         sensor diagnostics: I2C scan + one sonar ping
 *    L,<text>                  text for the bottom LCD row (mission status)
 *    O                         open the compartment doors
 *    C                         close the compartment doors
 *    H,<0|1>                   doors are physically at closed(0)/open(1) —
 *                              resync belief to reality, move nothing
 *  Board → Pi:
 *    S,READY,<ver>             boot complete
 *    S,PONG                    ping reply
 *    S,IMU,<name>,0x<addr>     IMU detected at boot
 *    S,INA219,0x40             power monitor detected
 *    S,LCD,0x27                display detected
 *    S,MFRC522,0x<ver>         RFID reader detected
 *    B,<volts>,<amps>          battery/bus power, 1 Hz
 *    S,GYROCAL,<gx>,<gy>,<gz>  gyro bias in rad/s after calibration
 *    I,ax,ay,az,gx,gy,gz       IMU 50 Hz — m/s^2 and rad/s, base_link axes
 *    U,<metres>                ultrasonic ~15 Hz; -1 means no echo
 *    R,<UID hex>               RFID tag scanned (uppercase, no separators)
 *    D,<0|1>                   compartment IR: 1 = something in there
 *    A,DOORS,<MOVING|OPEN|CLOSED>   door state; MOVING then the settled state
 *    E,<msg>                   error/diagnostic
 *
 * ─── Tuning ────────────────────────────────────────────────────────────────
 *  WHEEL_RADIUS / WHEEL_SEP / MAX_RPM : match urdf/properties.xacro
 *  MIN_PWM  : stiction floor. With v5 this is a real mechanical floor rather
 *             than the driver browning out, so it can go lower than before.
 *  INVERT_* : flip if a side runs backwards.
 */

#define FW_VERSION "v11-rfid-doors"

#include <Wire.h>
#include <SPI.h>

// Override the MFRC522 library's 4 MHz default. MUST come before MFRC522.h,
// which defines it inside an #ifndef.
//
// The reader is a 3.3 V part driven by 5 V Mega outputs down dupont wire, with
// no level shifter. VersionReg is a constant, so reading it repeatedly at a
// range of clocks measures the link directly — that is what `Z`'s diagRfidSpi
// does. Measured 2026-07-31, 20 reads at each clock:
//
//     4000 kHz -> 0x02, 18/19 differing
//     2000 kHz -> 0x04, 16/19
//     1000 kHz -> 0x04,  7/19
//      500 kHz -> 0x82, 10/19
//      250 kHz -> 0x82,  0/19      <- chosen
//      125 kHz -> 0x82,  1/19
//
// Monotonically better as the clock slows, which is the signature of signal
// integrity rather than a dead part. 250 kHz is the fastest clock that read
// clean. The cost is irrelevant here: a card poll is a few hundred bytes and
// only ever runs with the robot parked.
//
// THIS IS A MITIGATION, NOT A FIX. Note the value it settles on, 0x82, is not
// a documented MFRC522 version (0x91/0x92 genuine, 0x12/0x88 clone), and
// TReloadReg still does not read back what was written to it — so the link is
// quieter but not proven correct. Fit the level shifter (or 1k/2k dividers) on
// SCK/MOSI/SS/RST that the pin map calls for before trusting this reader.
#define MFRC522_SPICLOCK (250000u)
#include <MFRC522.h>
#include <Servo.h>
#include <EEPROM.h>

// ─── Motor driver pins (2× L298N, one per SIDE) ────────────────────────────
// All twelve verified PASS by pin_health.ino before being chosen.
#define FL_EN   7    // L298N-LEFT  ENA — front-left
#define FL_INA 30
#define FL_INB 31
#define RL_EN   8    // L298N-LEFT  ENB — rear-left
#define RL_INA 32
#define RL_INB 33
#define FR_EN  11    // L298N-RIGHT ENA — front-right
#define FR_INA 34
#define FR_INB 35
#define RR_EN  12    // L298N-RIGHT ENB — rear-right
#define RR_INA 36
#define RR_INB 37

// ─── Ultrasonic pins (HC-SR04) ─────────────────────────────────────────────
#define SONAR_TRIG 40
#define SONAR_ECHO 41

// ─── RFID pins (MFRC522, hardware SPI) ─────────────────────────────────────
// MISO=D50 MOSI=D51 SCK=D52 are the Mega's fixed hardware SPI pins and are not
// selectable. The module is a 3.3 V part: SCK/MOSI/SS/RST must go through a
// level shifter or divider, and it must be powered from 3.3 V, not 5 V. MISO
// needs nothing — 3.3 V already reads as HIGH on a 5 V input.
#define RFID_SS   53
#define RFID_RST  49

// ─── IR compartment sensor ─────────────────────────────────────────────────
#define IR_PIN    43

// ─── Door servo pins (2× SG90) ─────────────────────────────────────────────
// The Servo library claims Timer5 no matter which pins are attached, and
// Timer5 is exactly what drives hardware PWM on D44/D45/D46 — so putting the
// servos here costs nothing that was not already lost. The four motor enables
// (D7/D8/D11/D12) are plain DC digitalWrite outputs, not analogWrite, so the
// drive train is untouched by this.
// As wired and confirmed by the user 2026-07-31: LEFT door on D44, RIGHT on
// D45. (Briefly swapped earlier on a guess from the open/close asymmetry —
// that guess was wrong, and with the left servo dead it would have driven the
// surviving RIGHT servo to the left door's 175 close angle, 25 degrees past
// its stop. Do not re-swap these without physical confirmation.)
#define SERVO_L   44
#define SERVO_R   45

// The LEFT servo was replaced and re-enabled on 2026-08-01. Set this to 0 again
// if it ever fails: a dead servo still draws stall current when driven, and
// that is what repeatedly collapsed the 5 V rail and stopped the Mega booting
// mid-test. With it 0, doorTick moves only the right arm while the door STATE
// machine and its A,DOORS acks behave normally, which keeps the delivery
// mission FSM fully testable.
//
// The replacement was verified BEFORE being enabled here, with the arm
// UNCOUPLED from the door, using fw_testing/left_servo_probe. That sketch
// drives only D44, commands nothing in setup(), and makes its first attach at
// 90 deg -- identical to Servo::attach()'s own 1500 us default -- so the horn
// cannot lunge while its true orientation is still unknown. The probe confirmed
// this servo reaches LEFT_CLOSE_DEG 175 without grinding, and that 175 is
// genuinely the closed orientation for how the new horn is mounted, so no
// endpoint swap was needed. Note 175 deg is 2348 us, only 2 us inside the
// SERVO_US_MAX clamp -- there is no margin left at that end.
//
// Verified after enabling: two open/close cycles through /doors/open and
// /doors/close, both arms driven together, with no reset and no brownout.
#define DOOR_LEFT_PRESENT  1

// ─── Robot parameters (match URDF properties.xacro) ───────────────────────
#define WHEEL_RADIUS    0.065f
#define WHEEL_SEP       0.330f
#define MAX_RPM         100
#define MAX_SPEED_MPS   (MAX_RPM / 60.0f * 2.0f * 3.14159265f * WHEEL_RADIUS)

// ─── Drive shaping ─────────────────────────────────────────────────────────
// Stiction floor. Measured on the floor with all four wheels loaded: PWM 82
// (0.22 m/s commanded) could not break static friction, while 187 (0.5 m/s)
// drove fine. 90 is the lowest duty that reliably keeps the robot rolling once
// it is already moving — starting from rest is handled by the kick below.
#define MIN_PWM         90
#define INVERT_LEFT      0
#define INVERT_RIGHT     0

// ─── Skid-steer turn authority ─────────────────────────────────────────────
// A differential-drive mix (v = lin ± ang * track/2) assumes wheels that can
// pivot freely. A 4WD skid-steer cannot: to rotate, all four wheels must scrub
// sideways across the floor, and that scrub resists the turn hard. The honest
// differential mix therefore under-drives rotation badly — 0.9 rad/s produced
// only ±0.148 m/s of wheel-speed difference, which barely moved the robot.
//
// TURN_GAIN widens the *effective* track to compensate. 2.0 is a typical
// starting point for a 4WD skid-steer on hard floor; raise it if turns are
// still lazy, lower it if the robot over-rotates versus the commanded yaw.
//
// This deliberately breaks the "commanded rad/s == actual rad/s" contract.
// That is fine here: odometry comes from the LiDAR (rf2o), not from these
// numbers, so the map stays correct either way. It would NOT be acceptable if
// wheel odometry were feeding the EKF.
#define TURN_GAIN       2.0f

// ─── Pivot boost ───────────────────────────────────────────────────────────
// One gain cannot serve both jobs.
//
//   Following a path, the controller issues small heading corrections while
//   cruising. Those need a GENTLE gain — at TURN_GAIN 20 a 0.05 rad/s trim
//   would reverse one whole side and the robot would lurch down the path.
//
//   Turning on the spot needs a HUGE gain. Measured: rotation only happens
//   near full PWM (PWM 162 -> 0.057 rad/s, PWM 255 -> 0.220 rad/s), because
//   all four wheels must scrub sideways. With TURN_GAIN 2.0, Nav2 commanding
//   its 0.20 rad/s limit produced PWM 106 — nowhere near enough, so the robot
//   simply did not turn and Nav2's spin recovery timed out.
//
// So blend: full pivot authority when barely translating, normal arc gain once
// rolling. pivot_frac goes 1 -> 0 as |lin| rises to PIVOT_LIN_REF.
//
// SIZING THIS GAIN (v10, 2026-07-27): it must map Nav2's MAXIMUM angular
// command onto full PWM — no more, no less.
//
//   gain = MAX_SPEED_MPS / (max_ang * WHEEL_SEP/2)
//        = 0.6807 / (0.50 * 0.165) = 8.25  ->  8.0
//
// Nav2's angular ceiling was raised from 0.20 to 0.50 rad/s (see
// nav2_params.yaml), so the old 20.0 is now badly wrong in a way that matters:
// at gain 20 a 0.50 rad/s command asks for 2.5x full speed, the mix saturates,
// and EVERY turn command from 0.20 rad/s upward produces identical full-PWM
// output. That is a bang-bang controller — the robot slams into the turn and
// cannot modulate, which reads as "jerky" no matter what the smoother does.
//
// MEASURED AND CORRECTED 2026-07-27 (turntest.py, gyro as ground truth).
// The theoretical 8.0 restored the proportional band -- saturation gone,
// response monotonic -- but over-rotated at the top: 0.50 rad/s commanded
// produced 0.651 achieved (ratio 1.30), because the battery is healthier than
// when the original 0.22 rad/s full-PWM ceiling was measured. Nav2's
// controllers assume commanded ~= achieved, so a 30% overshoot shows up as
// heading oscillation on every rotate-to-heading.
//
// Scaled down to bring the working band (0.35-0.50 rad/s, where the rotation
// shim and the angular ceiling both live) onto ratio ~1.0. The response is not
// perfectly linear -- MIN_PWM's offset lifts the low end while stiction drags
// it back down -- so this trades a little accuracy at 0.15 rad/s, which the
// planner rarely asks for, for accuracy where it matters.
#define TURN_GAIN_PIVOT  6.5f
#define PIVOT_LIN_REF   0.10f

// ─── Stiction kick ─────────────────────────────────────────────────────────
// Static friction is far higher than rolling friction, so the duty needed to
// START moving is much higher than the duty needed to KEEP moving. Without
// this, any slow command just makes the motors buzz and stall.
//
// On a 0 -> moving transition (or a direction reversal) the channel is driven
// at KICK_PWM for KICK_MS, then falls back to the commanded duty. This lets
// /cmd_vel ask for genuinely slow speeds — which is what SLAM wants — without
// the robot refusing to start.
#define KICK_PWM       230
#define KICK_MS        180

// ─── Software PWM ──────────────────────────────────────────────────────────
#define SPWM_PERIOD_US 6000UL  // ~167 Hz

// ─── Timing ────────────────────────────────────────────────────────────────
#define CMD_TIMEOUT_MS   500
#define SERIAL_BAUD    115200
#define LINE_BUF_LEN       64

// ─── IMU (MPU6050 / MPU9250 / MPU6500) ─────────────────────────────────────
// One driver serves all three. The MPU9250 is an MPU6500 die plus an AK8963
// magnetometer, and the MPU6500 keeps the MPU6050's accel/gyro register map —
// so 0x3B..0x48 reads identically on every one of them. Only WHO_AM_I differs,
// and we use it purely to name the part in the boot banner. The magnetometer
// is deliberately NOT read: the EKF fuses yaw RATE, not heading, and an
// uncalibrated magnetometer near two L298Ns and four motors is worse than
// useless indoors.
// I2C bus clock. 400 kHz keeps the 14-byte burst near 350 us, but only works
// on clean wiring — see the `Z` diagnostic's per-clock burst test.
#define I2C_CLOCK_HZ   400000UL
#define MPU_ADDR_A     0x68   // AD0 low
#define MPU_ADDR_B     0x69   // AD0 high
#define MPU_REG_SMPLRT 0x19
#define MPU_REG_CONFIG 0x1A
#define MPU_REG_GYRO   0x1B
#define MPU_REG_ACCEL  0x1C
#define MPU_REG_DATA   0x3B
#define MPU_REG_PWR1   0x6B
#define MPU_REG_WHOAMI 0x75

// ±250 °/s and ±2 g — the most sensitive ranges, which is what we want: this
// robot tops out near 0.5 rad/s (29 °/s) and 0.2 m/s, nowhere near saturating.
#define GYRO_LSB_PER_DPS  131.0f
#define ACCEL_LSB_PER_G 16384.0f
#define DEG2RAD          0.0174532925f
#define GRAVITY          9.80665f

#define IMU_PERIOD_MS      20    // 50 Hz — matches the bridge's declared rate
#define GYRO_CAL_SAMPLES  400    // ~2 s of averaging at boot

// Axis remap: set these to match how the breakout is physically mounted.
// Defaults assume +X forward, +Y left, +Z up (REP-103, same as base_link).
// Use -1 to flip an axis, or swap the source indices to rotate the board.
// Order of the raw triplet is [X, Y, Z] as the chip reports it.
#define IMU_AXIS_X_SRC 0
#define IMU_AXIS_Y_SRC 1
#define IMU_AXIS_Z_SRC 2
#define IMU_AXIS_X_SGN  1.0f
#define IMU_AXIS_Y_SGN  1.0f
#define IMU_AXIS_Z_SGN  1.0f

// ─── INA219 power monitor ──────────────────────────────────────────────────
// High-side on the 3S pack: VIN+ from battery+, VIN- to the motor drivers and
// buck. Measures total draw of everything downstream.
#define INA_ADDR       0x40
#define INA_REG_CONFIG 0x00
#define INA_REG_BUSV   0x02
#define INA_REG_CURRENT 0x04
#define INA_REG_CALIB  0x05

// 32 V range, 320 mV shunt, 12-bit, continuous shunt+bus.
#define INA_CONFIG     0x399F
// cal = 0.04096 / (current_LSB * R_shunt), with the stock 0.1 ohm shunt and a
// 100 uA/bit LSB: 0.04096 / (0.0001 * 0.1) = 4096. Full scale is then
// 32767 * 100uA = 3.27 A -- ABOVE THAT THE CURRENT READING CLIPS. Four motors
// scrubbing through a skid-steer turn can exceed it. Bus VOLTAGE is measured
// independently and stays correct regardless, which is what the battery
// percentage is derived from, so a clipped current reading is not fatal.
// For honest current under stall, fit a 0.01 ohm shunt and set cal to 40960.
#define INA_CALIB      4096
#define INA_CURRENT_LSB 0.0001f   // A per bit
#define INA_BUSV_LSB    0.004f    // V per bit (register is bits 15..3)
#define INA_PERIOD_MS  1000       // 1 Hz — battery state changes slowly

// ─── 20x4 I2C LCD (PCF8574 backpack, HD44780 controller) ───────────────────
// Deliberately NOT LiquidCrystal_I2C: that library writes a whole line in one
// blocking call, which at ~135 us per character is several milliseconds — most
// of a software-PWM period, and the motors would stutter every refresh. This
// driver keeps a shadow buffer and pushes ONE changed character per tick, so
// the longest blocking I2C burst is a single character.
#define LCD_ADDR      0x27
#define LCD_COLS        20
#define LCD_ROWS         4
#define LCD_TICK_MS      3   // one character every 3 ms -> ~4.5% I2C duty
#define LCD_REFRESH_MS 500   // recompute the text twice a second

// PCF8574 pin mapping used by essentially every cheap backpack.
#define LCD_RS   0x01
#define LCD_RW   0x02
#define LCD_EN   0x04
#define LCD_BL   0x08   // backlight, held on

// ─── Ultrasonic (HC-SR04) ──────────────────────────────────────────────────
// 15 Hz. Faster than ~20 Hz risks hearing the PREVIOUS ping's echo bouncing
// back off a far wall and reporting a phantom close obstacle.
#define SONAR_PERIOD_MS    66
// Usable horizon. The HC-SR04 is specified to 4 m; 3.0 m is a sane local
// obstacle range for a 0.44 m robot in a 5.7 x 6.65 m room and leaves margin
// above the ~2.4 m this unit reads across the room. MUST match SONAR_MAX_M in
// arduino_bridge.py — the bridge reports max_range on no-echo so the costmap
// clears, and a mismatch would make the cone clear the wrong depth.
#define SONAR_MAX_M      3.0f
#define SPEED_OF_SOUND    343.0f
// Echo pulse width for the max range, plus margin: 3 m round trip is 17.5 ms.
// Beyond this we call it a no-echo rather than waiting for the sensor's own
// ~38 ms timeout, which would halve the update rate.
#define SONAR_ECHO_TIMEOUT_US 25000UL
#define SONAR_RISE_TIMEOUT_US 30000UL   // sensor never answered at all

// ─── RFID (MFRC522) ────────────────────────────────────────────────────────
// Poll rate. Nothing needs to be fast here: a human presenting a badge holds it
// against the reader for the best part of a second, and every poll is a stall
// (see the header). 10 Hz is far more than enough.
#define RFID_PERIOD_MS      200
// The reader's internal card-detect timeout, in units of its 25 us timer tick.
// 1000 ticks = 25 ms. This is deliberately the stock value: it is the setting
// that was proven on the bench to actually detect a tag, and shrinking it to
// save loop time is exactly the kind of "optimisation" that would silently
// stop cards being seen. The stall it causes is affordable because rfidTick
// refuses to run while the wheels turn or a door moves, so it only ever lands
// with the robot parked and idle.
#define RFID_TIMER_TICKS    1000
// Suppress repeats of the same UID. Without this a tag left sitting on the
// reader republishes at RFID_PERIOD_MS forever and floods the serial link.
#define RFID_REPEAT_MS      2000UL
// If the reader is absent at boot, retry this often rather than staying dead
// for the whole session — a momentary bad contact at power-up must not
// silently disable the one sensor the delivery mission cannot proceed without.
#define RFID_REPROBE_MS     5000UL

// ─── IR compartment sensor ─────────────────────────────────────────────────
#define IR_POLL_MS          20
#define IR_DEBOUNCE_MS      50UL
// Republish even when unchanged, so a subscriber that starts late (the mission
// FSM connects long after boot) learns the current state without waiting for
// someone to physically disturb the compartment.
#define IR_REFRESH_MS       2000UL
// FC-51-style IR obstacle modules pull their output LOW when something is in
// front of them. Flip this if yours is the active-high sort. The pin is held
// INPUT_PULLUP so an unplugged sensor reads "empty" rather than jamming the
// mission in WAIT_PACKAGE_REMOVAL forever.
#define IR_OCCUPIED_LEVEL   LOW

// ─── Door servos ───────────────────────────────────────────────────────────
// Bench-calibrated 2026-07-30 with the arms fitted to the real doors. The two
// sides are NOT mirror images — each keeps its own pair. Do not "tidy" these
// into a shared sweep; the arms differ on the frame.
// EXACTLY the constants declared in ~/Arduino/sketch_jul30a_working/. These
// cost real time on the bench to find; do not "improve" them.
//
// An earlier version used 157 here, inferred from that sketch's sweep loop
// writing RIGHT_CLOSE + i for i up to 72. That was wrong: it drove the right
// door 7 degrees past its declared endpoint, into the frame.
#define LEFT_CLOSE_DEG    175
#define LEFT_OPEN_DEG      90
#define RIGHT_CLOSE_DEG    85
#define RIGHT_OPEN_DEG    150
// One servo frame. The SG90 latches exactly one pulse per 20 ms frame, so
// updating faster than this is wasted work.
#define DOOR_FRAME_MS      20UL
// Full-sweep travel time, set from the ANGULAR RATE the bench sketch proved,
// not from what feels responsive.
//
// That sketch steps 2 degrees per 60 ms = ~38 deg/s, constant. An earlier
// version of this firmware used 1200 ms for the left door's 85 degree sweep;
// because smoothstep peaks at 1.5x its average rate, that is ~106 deg/s at
// mid-travel — nearly 3x the proven speed — and it stripped the servo gears.
//
// Sizing rule: peak = 1.5 * sweep / travel, and peak must stay <= ~38 deg/s.
// For the 85 degree left sweep that needs travel >= 3.4 s. Any change to the
// angles above must be re-checked against this.
//
// This EXCEEDS the bridge's original 4 s door-ack timeout, so
// DOOR_ACK_TIMEOUT_S in arduino_bridge.py was raised to 8 s to match. The two
// must be changed together or every door service call fails on a timeout
// while the door is still moving perfectly well.
#define DOOR_TRAVEL_MS    3400UL
// A reversal mid-travel scales its duration by how far it actually has to go,
// so a small correction is quick — but never so quick that it becomes a jab.
#define DOOR_MIN_TRAVEL_MS 150UL
// How long to keep holding the endpoint before releasing the servos. Long
// enough for the arm to settle, short enough that nothing sits stalled.
#define DOOR_HOLD_MS       400UL
// Backstop only. The angle->pulse conversion uses the Servo library's own
// 544-2400 us mapping so the calibrated angles above keep their bench meaning;
// these bounds exist to stop a future mis-edit parking a servo against its
// internal end stop, where it grinds and stalls at ~700 mA.
#define SERVO_US_MIN      600
#define SERVO_US_MAX     2350

// EEPROM slots holding where the doors were last left.
//
// Without this the firmware assumes CLOSED at every boot. If the doors were
// actually left OPEN, the first attach() + write() of a commanded move drives
// the arms the entire sweep at the servo's full ~600 deg/s to reach the
// position the firmware only THINKS they are at — a slam identical to the one
// that broke a servo from setup(), just relocated into doorCommand(). Every
// flash resets the board, so this fires constantly during development.
//
// A door move is a rare event, so the ~100k EEPROM write endurance is ample.
#define EE_DOOR_MAGIC_ADDR  0
#define EE_DOOR_STATE_ADDR  1
#define EE_DOOR_MAGIC       0xD7

// ─── State ─────────────────────────────────────────────────────────────────
unsigned long last_cmd_ms = 0;
char          line_buf[LINE_BUF_LEN];
uint8_t       line_len = 0;
bool          raw_mode = false;   // M/W/Y latch; exempt from the watchdog

struct Channel {
    uint8_t       en, ina, inb;
    int8_t        dir;         // +1 forward, -1 reverse, 0 brake
    uint8_t       duty;        // 0..255
    bool          last_on;
    int8_t        last_dir;
    unsigned long kick_until;  // millis() deadline for the stiction kick
};

// Order matters: FL, FR, RL, RR — matches the W command argument order.
Channel CH[4] = {
    {FL_EN, FL_INA, FL_INB, 0, 0, false, 0, 0},
    {FR_EN, FR_INA, FR_INB, 0, 0, false, 0, 0},
    {RL_EN, RL_INA, RL_INB, 0, 0, false, 0, 0},
    {RR_EN, RR_INA, RR_INB, 0, 0, false, 0, 0},
};

// IMU state
uint8_t       imu_addr = 0;      // 0 = not detected; sensor code no-ops
float         gyro_bias[3] = {0.0f, 0.0f, 0.0f};   // rad/s, subtracted on read
unsigned long imu_next_ms = 0;

// INA219 state
bool          ina_present = false;
unsigned long ina_next_ms = 0;
float         ina_volts = 0.0f;
float         ina_amps = 0.0f;

// LCD state. `want` is what should be on screen, `shown` is what actually is;
// lcdTick pushes the difference one character at a time.
bool          lcd_present = false;
char          lcd_want[LCD_ROWS][LCD_COLS];
char          lcd_shown[LCD_ROWS][LCD_COLS];
uint8_t       lcd_scan = 0;        // flat index of the next cell to examine
int8_t        lcd_cursor_row = -1; // where the HD44780 cursor currently is
int8_t        lcd_cursor_col = -1;
unsigned long lcd_next_tick_ms = 0;
unsigned long lcd_next_refresh_ms = 0;

// Ultrasonic state machine
enum SonarState { SONAR_IDLE, SONAR_TRIGGER, SONAR_WAIT_RISE, SONAR_WAIT_FALL };
SonarState    sonar_state = SONAR_IDLE;
unsigned long sonar_next_ms = 0;
unsigned long sonar_phase_us = 0;   // when the current phase started
unsigned long sonar_echo_start_us = 0;

// RFID state
MFRC522       rfid(RFID_SS, RFID_RST);
bool          rfid_present = false;
bool          rfid_antenna = false;      // TxControlReg bits 0-1 confirmed set
unsigned long rfid_antenna_drops = 0;    // times the chip lost its config
uint8_t       rfid_version = 0;
unsigned long rfid_next_ms = 0;
char          rfid_last_uid[21] = "";      // up to a 10-byte UID in hex
unsigned long rfid_last_ms = 0;

// IR compartment state. ir_state is what has been reported; ir_candidate is
// what the pin currently reads, which only becomes ir_state once it has held
// still for IR_DEBOUNCE_MS.
bool          ir_state = false;
bool          ir_candidate = false;
unsigned long ir_stable_ms = 0;
unsigned long ir_next_ms = 0;
unsigned long ir_refresh_ms = 0;

// Door state. door_pos_* track the CURRENT interpolated angle so a command
// arriving mid-travel starts from where the arms actually are, not from the
// endpoint they were heading for.
enum DoorState { DOOR_CLOSED, DOOR_OPENING, DOOR_OPEN, DOOR_CLOSING };
Servo         servo_l, servo_r;
DoorState     door_state = DOOR_CLOSED;
float         door_pos_l = LEFT_CLOSE_DEG;
float         door_pos_r = RIGHT_CLOSE_DEG;
float         door_from_l = LEFT_CLOSE_DEG, door_to_l = LEFT_CLOSE_DEG;
float         door_from_r = RIGHT_CLOSE_DEG, door_to_r = RIGHT_CLOSE_DEG;
unsigned long door_start_ms = 0;
unsigned long door_travel_ms = DOOR_TRAVEL_MS;
unsigned long door_next_frame_ms = 0;
unsigned long door_release_ms = 0;   // 0 = nothing pending

// Loop-cost instrumentation. The whole risk of v11 is that a new blocking
// peripheral quietly steals time from the IMU and sonar — and it presents as
// BOTH rates falling by the same fraction, which is easy to misread as a
// measurement-window artifact. These make the cost visible in the `Z` diag
// instead of inferable, so the next person does not have to guess.
unsigned long loop_count = 0;
unsigned long loop_rate_ms = 0;
unsigned long loop_rate_hz = 0;      // completed loop() passes per second
unsigned long rfid_poll_us = 0;      // duration of the last card poll
unsigned long rfid_poll_max_us = 0;  // worst seen since boot

// ───────────────────────────────────────────────────────────────────────────
// Software PWM — call as often as possible from loop()
// ───────────────────────────────────────────────────────────────────────────
void spwmTick() {
    unsigned long phase = micros() % SPWM_PERIOD_US;
    unsigned long now = millis();
    for (uint8_t i = 0; i < 4; i++) {
        Channel &c = CH[i];

        // Apply the stiction kick if this channel just started moving.
        uint8_t duty = c.duty;
        if (c.kick_until) {
            if ((long)(now - c.kick_until) < 0) {
                if (duty > 0 && duty < KICK_PWM) duty = KICK_PWM;
            } else {
                c.kick_until = 0;   // expired
            }
        }

        bool on = (c.dir != 0) && (duty > 0) &&
                  (phase < (unsigned long)duty * SPWM_PERIOD_US / 255UL);
        if (on == c.last_on && c.dir == c.last_dir) continue;   // nothing to do
        c.last_on = on;
        c.last_dir = c.dir;
        if (c.dir > 0) {
            digitalWrite(c.ina, on ? HIGH : LOW);
            digitalWrite(c.inb, LOW);
        } else if (c.dir < 0) {
            digitalWrite(c.ina, LOW);
            digitalWrite(c.inb, on ? HIGH : LOW);
        } else {
            digitalWrite(c.ina, LOW);
            digitalWrite(c.inb, LOW);
        }
    }
}

void spwmDelay(unsigned long ms) {
    unsigned long t0 = millis();
    while (millis() - t0 < ms) spwmTick();
}

// ───────────────────────────────────────────────────────────────────────────
// Motor primitives
// ───────────────────────────────────────────────────────────────────────────
void setWheels(int fl, int fr, int rl, int rr) {
    int v[4] = {fl, fr, rl, rr};
    for (uint8_t i = 0; i < 4; i++) {
        int p = v[i];
        if (p > 255)  p = 255;
        if (p < -255) p = -255;
        int8_t  new_dir  = (p > 0) ? 1 : (p < 0 ? -1 : 0);
        uint8_t new_duty = (uint8_t)(p < 0 ? -p : p);

        // Kick only on a genuine standing start or a direction reversal —
        // /cmd_vel re-sends at 20 Hz, so re-arming every call would leave the
        // channel permanently at KICK_PWM and destroy speed control.
        if (new_dir != 0 && (CH[i].dir == 0 || CH[i].dir != new_dir))
            CH[i].kick_until = millis() + KICK_MS;

        CH[i].dir  = new_dir;
        CH[i].duty = new_duty;
    }
}

void stopAll() {
    setWheels(0, 0, 0, 0);
    spwmTick();
}

void setSides(int pwm_l, int pwm_r) {
#if INVERT_LEFT
    pwm_l = -pwm_l;
#endif
#if INVERT_RIGHT
    pwm_r = -pwm_r;
#endif
    // one board per side: both left channels on the LEFT driver, both right
    // channels on the RIGHT driver — see the wiring header.
    setWheels(pwm_l, pwm_r, pwm_l, pwm_r);
}

// Map a wheel-linear-velocity (m/s) to a signed PWM.
//
// Earlier versions clamped anything below MIN_PWM *up* to MIN_PWM. That made
// the whole bottom third of the velocity range collapse onto one PWM value, so
// ramping the command produced no change at the wheels and then a jump — which
// is exactly the "sudden, not smooth" behaviour. Instead, REMAP: the usable
// velocity range spans MIN_PWM..255 continuously, so every change in commanded
// velocity produces a proportional change at the wheel.
int velToPwm(float v) {
    float mag = v < 0 ? -v : v;
    if (mag < 0.005f) return 0;             // deadband — genuinely stopped
    if (mag > MAX_SPEED_MPS) mag = MAX_SPEED_MPS;
    float frac = mag / MAX_SPEED_MPS;       // 0..1
    int pwm = MIN_PWM + (int)(frac * (255.0f - MIN_PWM) + 0.5f);
    if (pwm > 255) pwm = 255;
    return v < 0 ? -pwm : pwm;
}

// ───────────────────────────────────────────────────────────────────────────
// IMU — MPU6050 / MPU9250, I2C
// ───────────────────────────────────────────────────────────────────────────
// Re-initialise the TWI peripheral. The AVR Wire master can be left holding
// the bus after a transaction that ended without a STOP — a failed
// endTransmission(false) does exactly that — and every subsequent transfer
// then NAKs (error 2) even though the device is perfectly healthy. Measured on
// this rig: reads fail 400/400 after init, and a bare Wire.begin() restores
// them to 19/20. Cheap to call, so we call it whenever a read fails.
void i2cRecover() {
    Wire.end();
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
}

void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(imu_addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Returns 0xFF on a bus error, which is not a valid WHO_AM_I for any part.
//
// Tries a repeated start first (the datasheet-correct way, and the only form
// this rig's part accepts), then falls back to a full stop for the benefit of
// clones that want it. Keep the order: the fallback must never be the one that
// silently wins, or burst reads elsewhere will behave differently.
uint8_t mpuReadReg(uint8_t addr, uint8_t reg) {
    for (uint8_t stop = 0; stop < 2; stop++) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        if (Wire.endTransmission(stop ? true : false) != 0) continue;
        if (Wire.requestFrom(addr, (uint8_t)1) != 1) continue;
        return Wire.read();
    }
    return 0xFF;
}

// One burst read of accel[3], temp, gyro[3]. Returns false on a bus error so
// the caller can skip publishing rather than emit zeros — a stuck-at-zero gyro
// looks to the EKF like a perfectly still robot, which is a dangerous lie.
// Why the last mpuReadRaw failed: 1..5 = Wire.endTransmission() code,
// 100+n = requestFrom returned n bytes instead of 14. 0 = no failure yet.
uint8_t last_i2c_err = 0;

// Measured behaviour of the MPU6500 clone on this rig, via the `Z` diagnostic:
//
//   reads back-to-back with no gap : 20/20
//   reads with a 5 ms gap between  :  0/20, always err 2 (address NAK)
//
// It is not the bus clock (identical at 400/200/100 kHz) and not the software
// PWM (a bare delay(5) fails the same way). The part simply stops ACKing after
// a few ms idle and needs to be poked awake — the FIRST transaction after a
// gap is the one that gets NAKed, and an immediate retry then succeeds.
//
// So: retry immediately, several times, with no delay in between. Do NOT add
// a delay between attempts and do NOT call Wire.end()/begin() here — both were
// tried and both make it worse, turning a one-shot NAK into a dead bus.
#define MPU_READ_TRIES 4

bool mpuReadRawOnce(int16_t *accel, int16_t *gyro) {
    Wire.beginTransmission(imu_addr);
    Wire.write(MPU_REG_DATA);
    // Repeated start, per the datasheet.
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) { last_i2c_err = err; return false; }
    uint8_t got = Wire.requestFrom(imu_addr, (uint8_t)14);
    if (got != 14) { last_i2c_err = 100 + got; return false; }
    // Each 16-bit word is read into named locals first. Writing
    // `(Wire.read() << 8) | Wire.read()` would be a byte-order bug: the
    // evaluation order of the two operands is unspecified in C++, so the
    // compiler is free to fetch the low byte first.
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t hi = Wire.read(), lo = Wire.read();
        accel[i] = (int16_t)(((uint16_t)hi << 8) | lo);
    }
    Wire.read(); Wire.read();          // temperature — unused
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t hi = Wire.read(), lo = Wire.read();
        gyro[i] = (int16_t)(((uint16_t)hi << 8) | lo);
    }
    return true;
}

bool mpuReadRaw(int16_t *accel, int16_t *gyro) {
    for (uint8_t t = 0; t < MPU_READ_TRIES; t++)
        if (mpuReadRawOnce(accel, gyro)) return true;
    return false;
}

// Probe both addresses, wake the part and configure it. Sets imu_addr on
// success; everything else no-ops while it stays 0, so a missing or unplugged
// IMU degrades to exactly the v9 behaviour instead of hanging the board.
void imuInit() {
    const uint8_t addrs[2] = {MPU_ADDR_A, MPU_ADDR_B};
    for (uint8_t i = 0; i < 2; i++) {
        uint8_t who = mpuReadReg(addrs[i], MPU_REG_WHOAMI);
        // 0x68 MPU6050 · 0x70 MPU6500 · 0x71 MPU9250 · 0x73 MPU9255
        if (who == 0x68 || who == 0x70 || who == 0x71 || who == 0x73) {
            imu_addr = addrs[i];
            const char *name = (who == 0x68) ? "MPU6050"
                             : (who == 0x70) ? "MPU6500"
                             : (who == 0x71) ? "MPU9250" : "MPU9255";
            // Wake from sleep and clock off the X gyro PLL — more stable than
            // the internal 8 MHz oscillator the part boots with.
            mpuWrite(MPU_REG_PWR1,   0x01);
            delay(50);
            // DLPF 3: ~44 Hz accel / 42 Hz gyro. The low-pass matters on a
            // skid-steer — wheel scrub puts a lot of high-frequency chatter
            // into the chassis that would otherwise reach the EKF as noise.
            mpuWrite(MPU_REG_CONFIG, 0x03);
            mpuWrite(MPU_REG_SMPLRT, 0x13);   // 1 kHz / (1+19) = 50 Hz
            mpuWrite(MPU_REG_GYRO,   0x00);   // ±250 °/s
            mpuWrite(MPU_REG_ACCEL,  0x00);   // ±2 g
            delay(50);
            // Clear any bus state left by the config writes before the first
            // burst read — see i2cRecover().
            i2cRecover();
            Serial.print(F("S,IMU,"));
            Serial.print(name);
            Serial.print(F(",0x"));
            Serial.println(imu_addr, HEX);
            return;
        }
    }
    Serial.println(F("E,no MPU found at 0x68/0x69 — check SDA=D20 SCL=D21"));
}

// Average the gyro at rest and store the result as a bias. The robot MUST be
// stationary. Motors are stopped first so their vibration does not poison the
// average, and the caller is expected not to be driving.
void gyroCalibrate() {
    if (!imu_addr) {
        Serial.println(F("E,gyro cal skipped — no IMU"));
        return;
    }
    stopAll();
    float sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t got = 0;
    int16_t a[3], g[3];

    for (uint16_t i = 0; i < GYRO_CAL_SAMPLES; i++) {
        if (mpuReadRaw(a, g)) {
            for (uint8_t k = 0; k < 3; k++) sum[k] += (float)g[k];
            got++;
        }
        spwmDelay(5);
    }
    if (got < GYRO_CAL_SAMPLES / 2) {
        Serial.print(F("E,gyro cal failed — got "));
        Serial.print(got);
        Serial.print('/');
        Serial.print(GYRO_CAL_SAMPLES);
        Serial.print(F(" samples, last i2c err "));
        Serial.println(last_i2c_err);
        return;
    }
    for (uint8_t k = 0; k < 3; k++)
        gyro_bias[k] = (sum[k] / got) / GYRO_LSB_PER_DPS * DEG2RAD;
    Serial.print(F("S,GYROCAL,"));
    Serial.print(gyro_bias[0], 5); Serial.print(',');
    Serial.print(gyro_bias[1], 5); Serial.print(',');
    Serial.println(gyro_bias[2], 5);
}

// Sample and emit one `I,` line if it is time. Non-blocking apart from the
// ~350 us I2C burst.
void imuTick() {
    if (!imu_addr) return;
    unsigned long now = millis();
    if ((long)(now - imu_next_ms) < 0) return;
    imu_next_ms = now + IMU_PERIOD_MS;

    // Never let a full TX buffer stall the PWM loop — drop the sample instead.
    // A 40-byte line needs 40 bytes of headroom.
    if (Serial.availableForWrite() < 48) return;

    int16_t a[3], g[3];
    if (!mpuReadRaw(a, g)) return;

    float acc[3], gyr[3];
    for (uint8_t k = 0; k < 3; k++) {
        acc[k] = (float)a[k] / ACCEL_LSB_PER_G * GRAVITY;
        gyr[k] = (float)g[k] / GYRO_LSB_PER_DPS * DEG2RAD - gyro_bias[k];
    }

    const uint8_t src[3] = {IMU_AXIS_X_SRC, IMU_AXIS_Y_SRC, IMU_AXIS_Z_SRC};
    const float   sgn[3] = {IMU_AXIS_X_SGN, IMU_AXIS_Y_SGN, IMU_AXIS_Z_SGN};

    Serial.print(F("I,"));
    for (uint8_t k = 0; k < 3; k++) {
        Serial.print(acc[src[k]] * sgn[k], 4);
        Serial.print(',');
    }
    for (uint8_t k = 0; k < 3; k++) {
        Serial.print(gyr[src[k]] * sgn[k], 5);
        if (k < 2) Serial.print(',');
    }
    Serial.println();
}

// ───────────────────────────────────────────────────────────────────────────
// INA219 power monitor
// ───────────────────────────────────────────────────────────────────────────
bool inaWrite16(uint8_t reg, uint16_t val) {
    Wire.beginTransmission(INA_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(val >> 8));
    Wire.write((uint8_t)(val & 0xFF));
    return Wire.endTransmission() == 0;
}

// Same repeated-start form the MPU needs, and the same immediate-retry policy:
// these two share a bus, and a NAK after an idle gap is a bus-level symptom,
// not an MPU-specific one.
bool inaRead16(uint8_t reg, int16_t *out) {
    for (uint8_t t = 0; t < 3; t++) {
        Wire.beginTransmission(INA_ADDR);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) continue;
        if (Wire.requestFrom(INA_ADDR, (uint8_t)2) != 2) continue;
        uint8_t hi = Wire.read(), lo = Wire.read();
        *out = (int16_t)(((uint16_t)hi << 8) | lo);
        return true;
    }
    return false;
}

void ina219Init() {
    Wire.beginTransmission(INA_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println(F("E,no INA219 at 0x40 — battery telemetry disabled"));
        ina_present = false;
        return;
    }
    // Calibration MUST be written before the current register reads anything
    // but zero; the chip derives current from the shunt voltage using it.
    if (!inaWrite16(INA_REG_CALIB, INA_CALIB) ||
        !inaWrite16(INA_REG_CONFIG, INA_CONFIG)) {
        Serial.println(F("E,INA219 config write failed"));
        ina_present = false;
        return;
    }
    ina_present = true;
    Serial.println(F("S,INA219,0x40"));
}

void ina219Tick() {
    if (!ina_present) return;
    unsigned long now = millis();
    if ((long)(now - ina_next_ms) < 0) return;
    ina_next_ms = now + INA_PERIOD_MS;
    if (Serial.availableForWrite() < 24) return;

    int16_t rawv, rawi;
    if (!inaRead16(INA_REG_BUSV, &rawv)) return;
    if (!inaRead16(INA_REG_CURRENT, &rawi)) return;

    // Bus voltage lives in bits 15..3; bit 1 is the conversion-ready flag and
    // bit 0 the maths-overflow flag, so the shift is not optional.
    ina_volts = (float)(rawv >> 3) * INA_BUSV_LSB;
    ina_amps = (float)rawi * INA_CURRENT_LSB;

    Serial.print(F("B,"));
    Serial.print(ina_volts, 3);
    Serial.print(',');
    Serial.println(ina_amps, 3);
}

// ───────────────────────────────────────────────────────────────────────────
// 20x4 LCD over PCF8574 — incremental, never blocks for more than one char
// ───────────────────────────────────────────────────────────────────────────
// Push one PCF8574 port state. Everything else is built from this.
void lcdRaw(uint8_t data) {
    Wire.beginTransmission(LCD_ADDR);
    Wire.write(data | LCD_BL);
    Wire.endTransmission();
}

// One 8-bit value as two 4-bit nibbles, each strobed by pulsing E. Sent as a
// single I2C transaction (4 port writes) so the whole character costs one
// start/stop rather than four — ~135 us instead of ~270 us.
void lcdSend(uint8_t value, uint8_t mode) {
    uint8_t hi = (value & 0xF0) | mode | LCD_BL;
    uint8_t lo = ((value << 4) & 0xF0) | mode | LCD_BL;
    Wire.beginTransmission(LCD_ADDR);
    Wire.write(hi | LCD_EN);
    Wire.write(hi);
    Wire.write(lo | LCD_EN);
    Wire.write(lo);
    Wire.endTransmission();
}

void lcdCmd(uint8_t c)  { lcdSend(c, 0); }
void lcdData(uint8_t c) { lcdSend(c, LCD_RS); }

void lcdInit() {
    Wire.beginTransmission(LCD_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println(F("E,no LCD at 0x27"));
        lcd_present = false;
        return;
    }
    delay(50);
    // HD44780 cold-start into 4-bit mode. These three 0x30 nibbles are the
    // documented wake-up sequence and are needed regardless of what mode the
    // controller happened to power up in.
    lcdRaw(0x00);
    delay(20);
    for (uint8_t i = 0; i < 3; i++) {
        lcdRaw(0x30 | LCD_EN); lcdRaw(0x30);
        delay(5);
    }
    lcdRaw(0x20 | LCD_EN); lcdRaw(0x20);   // now switch to 4-bit
    delay(5);

    lcdCmd(0x28);  delay(2);   // 4-bit, 2-line mode, 5x8 font
    lcdCmd(0x0C);  delay(2);   // display on, cursor off, blink off
    lcdCmd(0x06);  delay(2);   // entry mode: increment, no shift
    lcdCmd(0x01);  delay(3);   // clear (slow: needs >1.5 ms)

    for (uint8_t r = 0; r < LCD_ROWS; r++)
        for (uint8_t c = 0; c < LCD_COLS; c++) {
            lcd_want[r][c] = ' ';
            lcd_shown[r][c] = ' ';   // matches the clear we just issued
        }
    lcd_present = true;
    Serial.println(F("S,LCD,0x27"));
}

// Row start addresses in DDRAM for a 20x4 panel. Rows 2/3 continue rows 0/1,
// which is why they are not evenly spaced.
uint8_t lcdRowAddr(uint8_t row) {
    static const uint8_t A[4] = {0x00, 0x40, 0x14, 0x54};
    return A[row & 3];
}

// Copy text into the target buffer, space-padded. Does not touch the panel.
void lcdSetRow(uint8_t row, const char *text) {
    if (row >= LCD_ROWS) return;
    for (uint8_t c = 0; c < LCD_COLS; c++)
        lcd_want[row][c] = text[c] ? text[c] : ' ';
    // stop copying past the terminator but keep padding
    uint8_t i = 0;
    while (text[i] && i < LCD_COLS) i++;
    for (uint8_t c = i; c < LCD_COLS; c++) lcd_want[row][c] = ' ';
}

// Push at most ONE differing character. Called from loop(); the rate limit is
// what keeps the I2C duty low enough not to disturb the software PWM.
void lcdTick() {
    if (!lcd_present) return;
    unsigned long now = millis();
    if ((long)(now - lcd_next_tick_ms) < 0) return;
    lcd_next_tick_ms = now + LCD_TICK_MS;

    // Find the next cell that differs, scanning round-robin so no row starves.
    for (uint8_t n = 0; n < LCD_ROWS * LCD_COLS; n++) {
        uint8_t idx = (lcd_scan + n) % (LCD_ROWS * LCD_COLS);
        uint8_t r = idx / LCD_COLS, c = idx % LCD_COLS;
        if (lcd_want[r][c] == lcd_shown[r][c]) continue;

        // Only re-address when the cursor is not already in the right place;
        // consecutive characters then cost one transaction each.
        if (lcd_cursor_row != (int8_t)r || lcd_cursor_col != (int8_t)c) {
            lcdCmd(0x80 | (lcdRowAddr(r) + c));
            lcd_cursor_row = r;
            lcd_cursor_col = c;
        }
        lcdData((uint8_t)lcd_want[r][c]);
        lcd_shown[r][c] = lcd_want[r][c];
        lcd_cursor_col++;
        if (lcd_cursor_col >= LCD_COLS) { lcd_cursor_row = -1; }
        lcd_scan = (idx + 1) % (LCD_ROWS * LCD_COLS);
        return;
    }
}

// Recompute the power display. Row 3 is left alone — it belongs to the ROS
// `L,` command so the Pi can show mission state there.
void lcdRefresh() {
    if (!lcd_present) return;
    unsigned long now = millis();
    if ((long)(now - lcd_next_refresh_ms) < 0) return;
    lcd_next_refresh_ms = now + LCD_REFRESH_MS;

    char buf[LCD_COLS + 1];

    lcdSetRow(0, "OfficeMate     v10.1");

    if (ina_present) {
        // 3S li-ion: 9.0 V empty, 12.6 V full — same window arduino_bridge uses.
        int pct = (int)((ina_volts - 9.0f) / (12.6f - 9.0f) * 100.0f + 0.5f);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        char v[8];
        dtostrf(ina_volts, 5, 2, v);
        snprintf(buf, sizeof(buf), "Bus %sV  Bat %3d%%", v, pct);
        lcdSetRow(1, buf);

        char a[8], w[8];
        dtostrf(ina_amps, 5, 3, a);
        dtostrf(ina_volts * ina_amps, 5, 1, w);
        snprintf(buf, sizeof(buf), "Cur %sA %sW", a, w);
        lcdSetRow(2, buf);
    } else {
        // Say so rather than showing a plausible-looking zero.
        lcdSetRow(1, "Bus  --.--V  Bat --%");
        lcdSetRow(2, "no INA219 detected");
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Ultrasonic — HC-SR04, fully non-blocking
//
// Deliberately NOT pulseIn(): that busy-waits for up to 30 ms, which is five
// software-PWM periods and would visibly stutter the motors. Polling the echo
// pin from loop() costs nothing and loses no useful precision — sound covers
// 1 cm in 58 us, far longer than one loop iteration.
// ───────────────────────────────────────────────────────────────────────────
void sonarReport(float metres) {
    if (Serial.availableForWrite() < 16) return;
    Serial.print(F("U,"));
    Serial.println(metres, 3);
}

void sonarTick() {
    unsigned long now_us = micros();

    switch (sonar_state) {
    case SONAR_IDLE:
        if ((long)(millis() - sonar_next_ms) < 0) return;
        // 10 us trigger pulse. Blocking, but it is 0.17% of a PWM period.
        digitalWrite(SONAR_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(SONAR_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(SONAR_TRIG, LOW);
        sonar_phase_us = micros();
        // Schedule the next ping from the TRIGGER, not from echo completion.
        // Measuring a far object holds the echo high for ~15 ms, which would
        // otherwise stretch the period to 80 ms and drop the rate to ~12 Hz.
        sonar_next_ms = millis() + SONAR_PERIOD_MS;
        sonar_state = SONAR_WAIT_RISE;
        break;

    case SONAR_WAIT_RISE:
        if (digitalRead(SONAR_ECHO) == HIGH) {
            sonar_echo_start_us = now_us;
            sonar_state = SONAR_WAIT_FALL;
        } else if (now_us - sonar_phase_us > SONAR_RISE_TIMEOUT_US) {
            // Sensor never raised echo — unplugged or dead. Report no-echo;
            // the bridge turns -1 into max_range so the costmap CLEARS rather
            // than blocking, and its safety clamp ignores stale readings.
            sonarReport(-1.0f);
            sonar_state = SONAR_IDLE;
        }
        break;

    case SONAR_WAIT_FALL:
        if (digitalRead(SONAR_ECHO) == LOW) {
            unsigned long width = now_us - sonar_echo_start_us;
            float m = (float)width * SPEED_OF_SOUND / 2.0f / 1000000.0f;
            sonarReport(m > SONAR_MAX_M ? -1.0f : m);
            sonar_state = SONAR_IDLE;
        } else if (now_us - sonar_echo_start_us > SONAR_ECHO_TIMEOUT_US) {
            // Echo still high past the max-range flight time: nothing within
            // range. Report no-echo instead of waiting out the sensor's own
            // ~38 ms timeout, which would halve the update rate.
            sonarReport(-1.0f);
            sonar_state = SONAR_IDLE;
        }
        break;

    default:
        sonar_state = SONAR_IDLE;
        break;
    }
}

// ───────────────────────────────────────────────────────────────────────────
// RFID — MFRC522 on hardware SPI
//
// Every poll of this reader is a blocking wait on its internal timer, so both
// defences described in the header are load-bearing: the timer is cut to ~2 ms
// in rfidProbe, and rfidTick refuses to run at all while any wheel is turning.
// ───────────────────────────────────────────────────────────────────────────
bool motorsBusy() {
    for (uint8_t i = 0; i < 4; i++) {
        if (CH[i].dir != 0) return true;
    }
    return false;
}

// Write the working configuration WITHOUT a soft reset.
//
// This exists because the chip loses its configuration constantly (see
// rfidTick), and the obvious repair — calling PCD_Init() again — blocks for
// ~89 ms. At a 200 ms poll that is nearly half of all wall time, which
// starved the 20 ms door frames badly enough that /doors/close missed the
// bridge's 4 s ack window. These are ~10 register writes, about 1 ms at
// 250 kHz, and restore everything a card poll depends on.
void rfidConfigure() {
    // 25 ms card-detect timeout. Deliberately the same value that was proven
    // to detect a card on the bench — do not shrink it to save loop time
    // without re-testing an actual tag.
    rfid.PCD_WriteRegister(MFRC522::TModeReg, 0x80);
    rfid.PCD_WriteRegister(MFRC522::TPrescalerReg, 0xA9);
    rfid.PCD_WriteRegister(MFRC522::TReloadRegH, (uint8_t)(RFID_TIMER_TICKS >> 8));
    rfid.PCD_WriteRegister(MFRC522::TReloadRegL, (uint8_t)(RFID_TIMER_TICKS & 0xFF));
    rfid.PCD_WriteRegister(MFRC522::TxASKReg, 0x40);
    rfid.PCD_WriteRegister(MFRC522::ModeReg, 0x3D);

    // Switch the antenna on and CONFIRM it, retrying immediately on failure.
    //
    // `PCD_AntennaOn()` sets bits 0-1 of TxControlReg and checks nothing. If
    // that write does not stick, the reader still answers every register read,
    // reports no error anywhere, and simply never sees a card — the most
    // misleading failure on this board, because everything looks healthy.
    rfid_antenna = false;
    for (uint8_t t = 0; t < 4 && !rfid_antenna; t++) {
        uint8_t v = rfid.PCD_ReadRegister(MFRC522::TxControlReg);
        rfid.PCD_WriteRegister(MFRC522::TxControlReg, v | 0x03);
        rfid_antenna =
            (rfid.PCD_ReadRegister(MFRC522::TxControlReg) & 0x03) == 0x03;
    }
}

// Probe for the reader and, if it answers, put it into a state where a poll is
// cheap. Also used as the periodic re-probe when the reader is missing.
void rfidProbe(bool announce) {
    rfid.PCD_Init();
    rfid_version = rfid.PCD_ReadRegister(MFRC522::VersionReg);

    // 0x00 and 0xFF are the two "nothing is driving MISO" readings — an absent
    // module, a dead one, or SPI wired wrong. Genuine parts report 0x91/0x92;
    // the common clones report 0x12 or 0x88 and work fine, so anything else is
    // accepted rather than rejected on version alone.
    if (rfid_version == 0x00 || rfid_version == 0xFF) {
        rfid_present = false;
        if (announce) Serial.println(F("E,no MFRC522"));
        return;
    }

    // MUST come after PCD_Init, which would otherwise overwrite it.
    rfidConfigure();
    if (!rfid_antenna && announce) Serial.println(F("E,rfid antenna OFF"));

    rfid_present = true;
    if (announce) {
        Serial.print(F("S,MFRC522,0x"));
        if (rfid_version < 0x10) Serial.print('0');
        Serial.println(rfid_version, HEX);
    }
}

void rfidTick() {
    // Never poll while driving: it would stall the software PWM, and a tag is
    // only ever presented with the robot parked at the dropoff anyway.
    if (motorsBusy()) return;

    // Nor while a door is moving. A card poll blocks for up to the reader's
    // 25 ms timeout, which is more than one 20 ms servo frame, and the doors
    // must reach their endpoint inside the bridge's 4 s ack window. Nothing is
    // lost: the tag has already been read by the time a door is asked to move.
    if (door_state == DOOR_OPENING || door_state == DOOR_CLOSING) return;

    unsigned long now = millis();
    if ((long)(now - rfid_next_ms) < 0) return;

    if (!rfid_present) {
        rfid_next_ms = now + RFID_REPROBE_MS;
        rfidProbe(false);
        if (rfid_present) {
            Serial.print(F("S,MFRC522,0x"));
            if (rfid_version < 0x10) Serial.print('0');
            Serial.println(rfid_version, HEX);
        }
        return;
    }

    rfid_next_ms = now + RFID_PERIOD_MS;

    // Self-heal. Measured on this rig 2026-07-31: the reader loses its ENTIRE
    // configuration roughly every other poll — a scratch marker written to
    // TReloadRegL disappears at exactly the same moments the antenna bit does,
    // which is a whole-chip reset rather than one lost write. Root cause is
    // its 3.3 V supply sagging when the antenna driver draws current (fit a
    // 100 uF + 100 nF across the module's 3.3 V/GND, or give it a real
    // regulator).
    //
    // Repair with rfidConfigure(), NOT rfidProbe(): the latter calls
    // PCD_Init(), which blocks ~89 ms, and at this poll rate that consumed
    // most of the loop and starved doorTick's 20 ms frames until
    // /doors/close missed the bridge's 4 s ack window.
    if ((rfid.PCD_ReadRegister(MFRC522::TxControlReg) & 0x03) != 0x03) {
        rfid_antenna_drops++;
        rfidConfigure();
        if (!rfid_antenna) return;      // still down; try again next poll
    }

    unsigned long t0 = micros();
    bool present = rfid.PICC_IsNewCardPresent();
    rfid_poll_us = micros() - t0;
    if (rfid_poll_us > rfid_poll_max_us) rfid_poll_max_us = rfid_poll_us;

    if (!present) return;
    if (!rfid.PICC_ReadCardSerial()) return;

    static const char HEXC[] = "0123456789ABCDEF";
    char uid[21];
    uint8_t n = rfid.uid.size;
    if (n > 10) n = 10;                 // cannot overflow uid[] whatever it says
    for (uint8_t i = 0; i < n; i++) {
        uid[i * 2]     = HEXC[rfid.uid.uidByte[i] >> 4];
        uid[i * 2 + 1] = HEXC[rfid.uid.uidByte[i] & 0x0F];
    }
    uid[n * 2] = '\0';

    // Stop talking to this card either way — leaving it selected blocks the
    // next detection, so a second scan of the same tag would never be seen.
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();

    bool same = (strcmp(uid, rfid_last_uid) == 0);
    if (same && (now - rfid_last_ms) < RFID_REPEAT_MS) return;

    strcpy(rfid_last_uid, uid);
    rfid_last_ms = now;

    Serial.print(F("R,"));
    Serial.println(uid);
}

// ───────────────────────────────────────────────────────────────────────────
// IR compartment sensor — is there still a package in there?
// ───────────────────────────────────────────────────────────────────────────
void irReport() {
    if (Serial.availableForWrite() < 8) return;
    Serial.print(F("D,"));
    Serial.println(ir_state ? 1 : 0);
    ir_refresh_ms = millis() + IR_REFRESH_MS;
}

void irTick() {
    unsigned long now = millis();
    if ((long)(now - ir_next_ms) < 0) return;
    ir_next_ms = now + IR_POLL_MS;

    bool raw = (digitalRead(IR_PIN) == IR_OCCUPIED_LEVEL);
    if (raw != ir_candidate) {
        // Reading changed — restart the settling clock rather than believing it.
        ir_candidate = raw;
        ir_stable_ms = now;
    } else if (raw != ir_state && (now - ir_stable_ms) >= IR_DEBOUNCE_MS) {
        ir_state = raw;
        irReport();
        return;
    }

    if ((long)(now - ir_refresh_ms) >= 0) irReport();
}

// ───────────────────────────────────────────────────────────────────────────
// Door servos — non-blocking, time-interpolated, smoothstep-eased
// ───────────────────────────────────────────────────────────────────────────
// Angle -> pulse using the Servo library's own 0-180 => 544-2400 us mapping,
// so the bench-calibrated angles keep exactly the meaning they had there, then
// clamped as a backstop (see SERVO_US_MIN/MAX).
int doorAngleToUs(float deg) {
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    int us = (int)(544.0f + deg * (2400.0f - 544.0f) / 180.0f + 0.5f);
    if (us < SERVO_US_MIN) us = SERVO_US_MIN;
    if (us > SERVO_US_MAX) us = SERVO_US_MAX;
    return us;
}

// Remember where the doors ended up, so the next boot does not have to guess.
// Only called when a move settles, and EEPROM.update skips the write when the
// byte is unchanged.
void doorSave() {
    EEPROM.update(EE_DOOR_MAGIC_ADDR, EE_DOOR_MAGIC);
    EEPROM.update(EE_DOOR_STATE_ADDR, door_state == DOOR_OPEN ? 1 : 0);
}

// Restore the last known door position at boot. A missing or wrong magic byte
// means a blank/foreign EEPROM, in which case CLOSED is the safe assumption:
// the doors rest closed, and being wrong that way commands a move AWAY from
// the stop rather than into it.
void doorRestore() {
    if (EEPROM.read(EE_DOOR_MAGIC_ADDR) != EE_DOOR_MAGIC) return;
    if (EEPROM.read(EE_DOOR_STATE_ADDR) == 1) {
        door_state = DOOR_OPEN;
        door_pos_l = LEFT_OPEN_DEG;
        door_pos_r = RIGHT_OPEN_DEG;
    }
}

void doorReport() {
    const __FlashStringHelper *s;
    switch (door_state) {
        case DOOR_OPEN:   s = F("OPEN");   break;
        case DOOR_CLOSED: s = F("CLOSED"); break;
        default:          s = F("MOVING"); break;
    }
    Serial.print(F("A,DOORS,"));
    Serial.println(s);
}

// Servos are attached only while a door is actually moving.
//
// Two reasons, both learned the hard way. (1) An attached servo starts pulsing
// the instant it is attached, so attaching at boot commands a position before
// anyone asked for one — that is what crashed the left arm into the frame.
// (2) A held SG90 fights any mechanical mismatch forever, buzzing and drawing
// current; released, it is limp and draws nothing. These doors are light and
// stay put on gearbox friction alone.
// ORDER IS LOAD-BEARING: set the pulse width BEFORE attaching.
//
// Servo::attach() begins pulsing at DEFAULT_PULSE_WIDTH — 1500 us, i.e. 90
// degrees — and does NOT touch the stored pulse value. So attaching first and
// writing the real position afterwards commands 90 degrees for at least one
// frame, and the arm lunges there at the servo's full ~600 deg/s before the
// eased motion ever starts.
//
// That is what made OPEN work and CLOSE kill the board. Opening, the right
// arm sits at 85 deg, so the lunge to 90 is 5 degrees and harmless. Closing,
// it sits at 150, so attach() drove it 60 degrees instantly while the left arm
// was being driven too — and the combined stall current browned out the Mega
// before the MOVING ack could even be printed.
//
// Writing first is safe because Servo assigns servoIndex in its CONSTRUCTOR,
// not in attach(), so writeMicroseconds() is honoured on a detached servo and
// attach() then picks up that value instead of the 1500 us default.
void doorAttach() {
#if DOOR_LEFT_PRESENT
    servo_l.writeMicroseconds(doorAngleToUs(door_pos_l));
    if (!servo_l.attached()) servo_l.attach(SERVO_L);
#endif
    servo_r.writeMicroseconds(doorAngleToUs(door_pos_r));
    if (!servo_r.attached()) servo_r.attach(SERVO_R);
}

void doorDetach() {
    if (servo_l.attached()) servo_l.detach();
    if (servo_r.attached()) servo_r.detach();
}

void doorApply() {
    // Never pulse a servo that is not meant to be moving.
#if DOOR_LEFT_PRESENT
    if (!servo_l.attached()) return;
    servo_l.writeMicroseconds(doorAngleToUs(door_pos_l));
#endif
    if (!servo_r.attached()) return;
    servo_r.writeMicroseconds(doorAngleToUs(door_pos_r));
}

// Declare where the doors physically are, WITHOUT moving them.
//
// The recovery path for the one case doorRestore() cannot cover: the arms have
// been moved by hand, or the EEPROM does not yet know about them, so the
// firmware's belief and reality disagree. Commanding a move in that state
// slams the arms the full sweep. This resyncs belief to reality instead.
// Servos are left detached, so nothing moves as a result of this.
void doorAssume(bool open) {
    door_state = open ? DOOR_OPEN : DOOR_CLOSED;
    door_pos_l = open ? LEFT_OPEN_DEG  : LEFT_CLOSE_DEG;
    door_pos_r = open ? RIGHT_OPEN_DEG : RIGHT_CLOSE_DEG;
    doorSave();
    doorReport();
}

void doorCommand(bool open) {
    DoorState settled = open ? DOOR_OPEN : DOOR_CLOSED;

    // Already there — re-ack so the bridge's service call completes instead of
    // sitting out its 4 s timeout waiting for a transition that cannot happen.
    if (door_state == settled) {
        doorReport();
        return;
    }

    door_from_l = door_pos_l;
    door_from_r = door_pos_r;
    door_to_l = open ? LEFT_OPEN_DEG  : LEFT_CLOSE_DEG;
    door_to_r = open ? RIGHT_OPEN_DEG : RIGHT_CLOSE_DEG;

    // Scale the travel time by how far there actually is to go, so reversing
    // mid-sweep does not spend the full duration crawling a few degrees.
    float span_l = fabs(door_to_l - door_from_l) / fabs((float)(LEFT_OPEN_DEG  - LEFT_CLOSE_DEG));
    float span_r = fabs(door_to_r - door_from_r) / fabs((float)(RIGHT_OPEN_DEG - RIGHT_CLOSE_DEG));
    float frac = span_l > span_r ? span_l : span_r;
    if (frac > 1.0f) frac = 1.0f;
    door_travel_ms = (unsigned long)(DOOR_TRAVEL_MS * frac);
    if (door_travel_ms < DOOR_MIN_TRAVEL_MS) door_travel_ms = DOOR_MIN_TRAVEL_MS;

    door_start_ms = millis();
    door_next_frame_ms = door_start_ms;
    door_state = open ? DOOR_OPENING : DOOR_CLOSING;

    // Attach and immediately command the position the arms are ALREADY at, so
    // the servo's first pulse asks for no movement. Attaching without this
    // makes the servo snap to wherever its last commanded position was.
    doorAttach();
    doorApply();

    doorReport();          // MOVING
}

void doorTick() {
    if (door_state != DOOR_OPENING && door_state != DOOR_CLOSING) return;

    unsigned long now = millis();
    if ((long)(now - door_next_frame_ms) < 0) return;
    door_next_frame_ms = now + DOOR_FRAME_MS;

    float p = (float)(now - door_start_ms) / (float)door_travel_ms;
    if (p > 1.0f) p = 1.0f;
    // Smoothstep: zero velocity at both ends, so a long printed arm cannot
    // bounce against the door frame when it arrives.
    float e = p * p * (3.0f - 2.0f * p);

    door_pos_l = door_from_l + (door_to_l - door_from_l) * e;
    door_pos_r = door_from_r + (door_to_r - door_from_r) * e;
    doorApply();

    if (p >= 1.0f) {
        door_state = (door_state == DOOR_OPENING) ? DOOR_OPEN : DOOR_CLOSED;
        // Hold briefly so the arm settles at the endpoint before going limp,
        // then release (see doorAttach for why we do not hold indefinitely).
        door_release_ms = now + DOOR_HOLD_MS;
        doorSave();
        doorReport();
    }
}

void doorReleaseTick() {
    if (door_release_ms == 0) return;
    if ((long)(millis() - door_release_ms) < 0) return;
    door_release_ms = 0;
    doorDetach();
}

// ───────────────────────────────────────────────────────────────────────────
// Sensor diagnostics — `Z` command
//
// Answers the two questions you actually have when a sensor is silent:
// "is ANYTHING on the I2C bus?" and "is the echo pin doing anything at all?".
// Distinguishes a wiring/power fault from a wrong-address or dead-part fault.
// ───────────────────────────────────────────────────────────────────────────
void diagI2C() {
    // Idle level of the bus lines with pull-ups engaged. Both should read
    // HIGH. A line stuck LOW means no pull-up, a short, or a wedged device;
    // that is a wiring fault, not an addressing one.
    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(50);
    int sda = digitalRead(SDA), scl = digitalRead(SCL);
    Serial.print(F("E,DIAG i2c idle SDA="));
    Serial.print(sda ? F("HIGH") : F("LOW (fault)"));
    Serial.print(F(" SCL="));
    Serial.println(scl ? F("HIGH") : F("LOW (fault)"));
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);

    uint8_t found = 0;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("E,DIAG i2c device at 0x"));
            Serial.println(a, HEX);
            found++;
        }
    }
    Serial.print(F("E,DIAG i2c devices found: "));
    Serial.println(found);

    // Raw WHO_AM_I from both candidate addresses. If a device answered the
    // scan above but reports an ID we do not recognise, this is the number
    // that tells us what to add to imuInit()'s accept list.
    for (uint8_t i = 0; i < 2; i++) {
        uint8_t a = i ? MPU_ADDR_B : MPU_ADDR_A;
        Serial.print(F("E,DIAG whoami 0x"));
        Serial.print(a, HEX);
        Serial.print(F(" -> 0x"));
        Serial.println(mpuReadReg(a, MPU_REG_WHOAMI), HEX);
    }

    // Burst-read reliability against what happens BETWEEN reads. Reads that
    // work back-to-back but fail when spaced apart point at an interaction
    // with whatever runs in the gap, not at the bus itself.
    if (imu_addr) {
        int16_t a[3], g[3];
        for (uint8_t mode = 0; mode < 3; mode++) {
            uint8_t ok = 0;
            for (uint8_t i = 0; i < 20; i++) {
                if (mpuReadRaw(a, g)) ok++;
                if (mode == 1) delay(5);         // plain delay
                else if (mode == 2) spwmDelay(5); // delay + software PWM
            }
            Serial.print(F("E,DIAG burst gap="));
            Serial.print(mode == 0 ? F("none") : (mode == 1 ? F("delay") : F("spwm")));
            Serial.print(F(": "));
            Serial.print(ok);
            Serial.print(F("/20 lasterr="));
            Serial.println(last_i2c_err);
        }
    }
}

// One blocking ping on an arbitrary trig/echo pair. Blocking is fine here: Z
// is a bench command and the motors are stopped. Returns the echo width in us,
// or 0 for no echo.
unsigned long diagPing(uint8_t trig, uint8_t echo) {
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    digitalWrite(trig, LOW);
    delayMicroseconds(4);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    unsigned long t0 = micros();
    while (digitalRead(echo) == LOW)
        if (micros() - t0 > 50000UL) return 0;
    unsigned long rise = micros();
    while (digitalRead(echo) == HIGH)
        if (micros() - rise > 60000UL) return 0;
    return micros() - rise;
}

void diagSonar() {
    stopAll();
    pinMode(SONAR_ECHO, INPUT);
    Serial.print(F("E,DIAG echo(D41) idle="));
    Serial.println(digitalRead(SONAR_ECHO) ? F("HIGH (stuck?)") : F("LOW (ok)"));

    // Try the configured orientation, then the swap. TRIG and ECHO being
    // crossed is by far the most common HC-SR04 wiring mistake and costs one
    // extra ping to rule in or out.
    struct { uint8_t trig, echo; const char *label; } tries[] = {
        {SONAR_TRIG, SONAR_ECHO, "TRIG=D40 ECHO=D41 (configured)"},
        {SONAR_ECHO, SONAR_TRIG, "TRIG=D41 ECHO=D40 (swapped)"},
    };
    for (uint8_t i = 0; i < 2; i++) {
        unsigned long w = diagPing(tries[i].trig, tries[i].echo);
        Serial.print(F("E,DIAG ping "));
        Serial.print(tries[i].label);
        if (w) {
            Serial.print(F(" -> width="));
            Serial.print(w);
            Serial.print(F("us = "));
            Serial.print((float)w * SPEED_OF_SOUND / 2.0f / 1000000.0f, 3);
            Serial.println(F(" m"));
        } else {
            Serial.println(F(" -> NO ECHO"));
        }
        delay(60);
    }
    // Restore the configured direction so the running state machine is sane.
    pinMode(SONAR_TRIG, OUTPUT);
    pinMode(SONAR_ECHO, INPUT);
    digitalWrite(SONAR_TRIG, LOW);
}

// RFID and IR, for the same reason as the I2C scan: tell a missing/miswired
// part from a merely idle one. A VersionReg of 0x00 or 0xFF means nothing is
// driving MISO — check power (3.3 V, not 5 V), SS/RST, and the level shifter.
// Read VersionReg at a range of SPI clocks, bypassing the library entirely.
//
// VersionReg is a constant, so this separates the two explanations for a
// reader that answers but never completes a transaction:
//   - stable and valid only at low clocks -> signal integrity, i.e. the
//     missing level shifter. Slowing down is a usable workaround.
//   - unstable or wrong at EVERY clock    -> not signal integrity. Suspect
//     power (the module needs 3.3 V), a broken part, or a wiring error.
// Valid answers are 0x91/0x92 (genuine) or 0x12/0x88 (common clones).
void diagRfidSpi() {
    static const uint32_t CLOCKS[] = {
        4000000UL, 2000000UL, 1000000UL, 500000UL, 250000UL, 125000UL
    };
    for (uint8_t c = 0; c < 6; c++) {
        uint8_t first = 0, diff = 0;
        for (uint8_t i = 0; i < 20; i++) {
            SPI.beginTransaction(SPISettings(CLOCKS[c], MSBFIRST, SPI_MODE0));
            digitalWrite(RFID_SS, LOW);
            // MFRC522 read frame: bit7 = 1 for read, address in bits 6..1.
            SPI.transfer(0x80 | ((0x37 << 1) & 0x7E));   // VersionReg = 0x37
            uint8_t v = SPI.transfer(0);
            digitalWrite(RFID_SS, HIGH);
            SPI.endTransaction();
            if (i == 0) first = v;
            else if (v != first) diff++;
        }
        Serial.print(F("E,rfid spi "));
        Serial.print(CLOCKS[c] / 1000UL);
        Serial.print(F("kHz ver 0x"));
        if (first < 0x10) Serial.print('0');
        Serial.print(first, HEX);
        Serial.print(F(" unstable "));
        Serial.print(diff);
        Serial.println(F("/19"));
    }
}

void diagRfidIr() {
    // VersionReg is a CONSTANT. Reading it 20 times and getting more than one
    // answer is proof of a marginal SPI link — which is the difference between
    // "no tag was presented" and "this reader can never work". Without this
    // check a flaky link is invisible: any single read looks plausible.
    uint8_t first = rfid.PCD_ReadRegister(MFRC522::VersionReg);
    uint8_t differing = 0;
    for (uint8_t i = 0; i < 20; i++) {
        if (rfid.PCD_ReadRegister(MFRC522::VersionReg) != first) differing++;
    }
    Serial.print(F("E,rfid VersionReg 0x"));
    if (first < 0x10) Serial.print('0');
    Serial.print(first, HEX);
    Serial.print(F(" unstable "));
    Serial.print(differing);
    Serial.print(F("/20 present "));
    Serial.println(rfid_present ? 1 : 0);

    // The antenna is the difference between "no tag was presented" and "this
    // reader physically cannot see one". drops counts whole-chip resets.
    uint8_t tx = rfid.PCD_ReadRegister(MFRC522::TxControlReg);
    Serial.print(F("E,rfid TxControl 0x"));
    Serial.print(tx, HEX);
    Serial.print((tx & 0x03) == 0x03 ? F(" ANTENNA-ON") : F(" ANTENNA-OFF"));
    Serial.print(F(" drops "));
    Serial.println(rfid_antenna_drops);

    // Read the card-detect timeout back. If these are not what rfidProbe wrote,
    // the write never landed and every poll costs the full 36 ms.
    Serial.print(F("E,rfid TReload "));
    Serial.print(rfid.PCD_ReadRegister(MFRC522::TReloadRegH));
    Serial.print('/');
    Serial.print(rfid.PCD_ReadRegister(MFRC522::TReloadRegL));
    // Split the same way rfidConfigure writes it, or a healthy 3/232 looks
    // like a mismatch against a 16-bit 1000.
    Serial.print(F(" want "));
    Serial.print((uint8_t)(RFID_TIMER_TICKS >> 8));
    Serial.print('/');
    Serial.println((uint8_t)(RFID_TIMER_TICKS & 0xFF));

    Serial.print(F("E,ir pin "));
    Serial.print(digitalRead(IR_PIN));
    Serial.print(F(" occupied "));
    Serial.println(ir_state ? 1 : 0);

    Serial.print(F("E,rfid poll last "));
    Serial.print(rfid_poll_us);
    Serial.print(F("us max "));
    Serial.print(rfid_poll_max_us);
    Serial.println(F("us"));

    Serial.print(F("E,loop "));
    Serial.print(loop_rate_hz);
    Serial.println(F(" Hz"));

    Serial.print(F("E,doors state "));
    Serial.print(door_state == DOOR_OPEN   ? F("OPEN")
               : door_state == DOOR_CLOSED ? F("CLOSED")
                                           : F("MOVING"));
    Serial.print(F(" L "));
    Serial.print(door_pos_l, 1);
    Serial.print(F(" R "));
    Serial.println(door_pos_r, 1);
}

void runDiag() {
    Serial.println(F("E,DIAG start"));
    diagI2C();
    diagSonar();
    diagRfidSpi();
    diagRfidIr();
    Serial.println(F("E,DIAG done"));
}

// ───────────────────────────────────────────────────────────────────────────
// Command handling
// ───────────────────────────────────────────────────────────────────────────
void handleDrive(char *args) {
    char *lin_s = strtok(args, ",");
    char *ang_s = strtok(NULL, ",");
    if (!lin_s || !ang_s) {
        Serial.println(F("E,bad V command"));
        return;
    }
    float lin = atof(lin_s);
    float ang = atof(ang_s);

    // Skid-steer mix about an effective (scrub-compensated) half-track.
    // Positive ang = turn left = right side faster forward, left side slower.
    // Gain blends from pivot authority (stopped) to arc gain (rolling).
    float alin = lin < 0 ? -lin : lin;
    float pivot_frac = 1.0f - (alin / PIVOT_LIN_REF);
    if (pivot_frac < 0.0f) pivot_frac = 0.0f;
    if (pivot_frac > 1.0f) pivot_frac = 1.0f;
    float gain = TURN_GAIN + pivot_frac * (TURN_GAIN_PIVOT - TURN_GAIN);
    float half_track = (WHEEL_SEP / 2.0f) * gain;
    float v_l = lin - ang * half_track;
    float v_r = lin + ang * half_track;

    // If the mix exceeds full speed, scale BOTH sides by the same factor
    // rather than clipping one. Clipping would flatten the speed difference
    // and turn a commanded arc into a straight line at high forward speed.
    float pk_l = v_l < 0 ? -v_l : v_l;
    float pk_r = v_r < 0 ? -v_r : v_r;
    float peak = pk_l > pk_r ? pk_l : pk_r;
    if (peak > MAX_SPEED_MPS) {
        float k = MAX_SPEED_MPS / peak;
        v_l *= k;
        v_r *= k;
    }

    raw_mode = false;
    last_cmd_ms = millis();
    int pwm_l = velToPwm(v_l);
    int pwm_r = velToPwm(v_r);
    setSides(pwm_l, pwm_r);

    // Diagnostic echo: report what actually arrived, but only when it CHANGES
    // (V arrives at 20 Hz, so echoing every one would flood the link).
    // arduino_bridge logs 'E' lines as warnings, so these land in the launch
    // log and prove whether ROS commands are reaching the board at all.
    static float last_lin = 9e9f, last_ang = 9e9f;
    if (fabs(lin - last_lin) > 0.001f || fabs(ang - last_ang) > 0.001f) {
        last_lin = lin;
        last_ang = ang;
        Serial.print(F("E,RX lin="));
        Serial.print(lin, 3);
        Serial.print(F(" ang="));
        Serial.print(ang, 3);
        Serial.print(F(" pwmL="));
        Serial.print(pwm_l);
        Serial.print(F(" pwmR="));
        Serial.println(pwm_r);
    }
}

void handleRaw(char *args) {
    char *l_s = strtok(args, ",");
    char *r_s = strtok(NULL, ",");
    if (!l_s || !r_s) {
        Serial.println(F("E,bad M command"));
        return;
    }
    raw_mode = true;
    last_cmd_ms = millis();
    setSides(atoi(l_s), atoi(r_s));
}

void handleWheels(char *args) {
    int v[4];
    for (uint8_t i = 0; i < 4; i++) {
        char *tok = strtok(i == 0 ? args : NULL, ",");
        if (!tok) {
            Serial.println(F("E,bad W command"));
            return;
        }
        v[i] = atoi(tok);
    }
    raw_mode = true;
    last_cmd_ms = millis();
    setWheels(v[0], v[1], v[2], v[3]);
}

// Retained for compatibility with motortest.py's `enall` mode. With v5 the
// enables are always DC high, so this is now just "wheel N at full".
void handleEnableAll(char *args) {
    int w = atoi(args);
    if (w < 0 || w > 3) { Serial.println(F("E,usage Y,<0-3>")); return; }
    int v[4] = {0, 0, 0, 0};
    v[w] = 255;
    raw_mode = true;
    last_cmd_ms = millis();
    setWheels(v[0], v[1], v[2], v[3]);
    Serial.print(F("S,ENALL,wheel="));
    Serial.println(w);
}

// Self-contained demo. Needs no host loop — send X once and watch.
void runDemo() {
    struct Step { int fl, fr, rl, rr; const char *label; };
    static const Step STEPS[] = {
        { 200,   0,   0,   0, "front-left"  },
        {   0, 200,   0,   0, "front-right" },
        {   0,   0, 200,   0, "rear-left"   },
        {   0,   0,   0, 200, "rear-right"  },
        { 200, 200, 200, 200, "all forward" },
        {-200,-200,-200,-200, "all reverse" },
    };
    Serial.println(F("S,DEMO,start"));
    for (uint8_t i = 0; i < sizeof(STEPS) / sizeof(STEPS[0]); i++) {
        Serial.print(F("S,DEMO,"));
        Serial.println(STEPS[i].label);
        setWheels(STEPS[i].fl, STEPS[i].fr, STEPS[i].rl, STEPS[i].rr);
        spwmDelay(2000);
        stopAll();
        spwmDelay(800);
    }
    Serial.println(F("S,DEMO,done"));
    raw_mode = false;
    last_cmd_ms = millis();
}

void handleLine(char *line) {
    switch (line[0]) {
        case 'V': if (line[1] == ',') handleDrive(line + 2);     break;
        case 'M': if (line[1] == ',') handleRaw(line + 2);       break;
        case 'W': if (line[1] == ',') handleWheels(line + 2);    break;
        case 'Y': if (line[1] == ',') handleEnableAll(line + 2); break;
        case 'X': runDemo();                                     break;
        case 'S':
            raw_mode = false;
            stopAll();
            break;
        case 'P':
            Serial.println(F("S,PONG"));
            break;
        case 'G':
            gyroCalibrate();
            break;
        case 'Z':
            runDiag();
            break;
        case 'O':
            doorCommand(true);
            break;
        case 'C':
            doorCommand(false);
            break;
        case 'H':   // "the doors are physically HERE" — believe it, move nothing
            if (line[1] == ',') doorAssume(line[2] == '1');
            break;
        case 'L':   // status text from ROS -> bottom LCD row
            if (line[1] == ',') lcdSetRow(3, line + 2);
            break;
        default:
            Serial.print(F("E,unknown cmd "));
            Serial.println(line[0]);
            break;
    }
}

void readSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (line_len > 0) {
                line_buf[line_len] = '\0';
                handleLine(line_buf);
                line_len = 0;
            }
        } else if (line_len < LINE_BUF_LEN - 1) {
            line_buf[line_len++] = c;
        } else {
            line_len = 0;   // overrun — drop the line
            Serial.println(F("E,line overrun"));
        }
    }
}

// ───────────────────────────────────────────────────────────────────────────
void setup() {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(CH[i].en,  OUTPUT);
        pinMode(CH[i].ina, OUTPUT);
        pinMode(CH[i].inb, OUTPUT);
        digitalWrite(CH[i].ina, LOW);
        digitalWrite(CH[i].inb, LOW);
        // Enables are parked HIGH for the whole run. Both INs low is brake, so
        // nothing turns until a command arrives.
        digitalWrite(CH[i].en, HIGH);
    }
    stopAll();

    pinMode(SONAR_TRIG, OUTPUT);
    pinMode(SONAR_ECHO, INPUT);
    digitalWrite(SONAR_TRIG, LOW);

    // INPUT_PULLUP, so an unplugged IR sensor reads HIGH — which with
    // IR_OCCUPIED_LEVEL == LOW means "empty". A floating pin that drifted to
    // "occupied" would hang the mission in WAIT_PACKAGE_REMOVAL forever.
    pinMode(IR_PIN, INPUT_PULLUP);

    // DELIBERATELY does not attach or drive the servos.
    //
    // An earlier version parked the doors here with a bare doorApply(), which
    // is an unrestrained full-speed jump to the closed angle. Every flash and
    // every board reset re-ran it, so an arm that happened to be open was
    // driven the whole sweep into the door frame at the SG90's ~600 deg/s and
    // stalled there. That destroyed the left servo, and a stalled SG90 pulling
    // ~700 mA dragged the 5 V rail down far enough that the board stopped
    // booting at all. The easing in doorTick did not help, because this path
    // bypassed it completely.
    //
    // A servo with no signal is limp and draws nothing, which is the right
    // state for an unknown door position. The doors rest closed mechanically,
    // so door_state starts CLOSED — but nothing is COMMANDED until the host
    // asks, and when it does, doorCommand eases over the full travel time
    // instead of stepping. See doorAttach().
    door_pos_l = LEFT_CLOSE_DEG;
    door_pos_r = RIGHT_CLOSE_DEG;
    door_state = DOOR_CLOSED;
    doorRestore();      // may correct the above to OPEN — see doorRestore()

    Serial.begin(SERIAL_BAUD);
    delay(200);

    // 400 kHz keeps the 14-byte burst read near 350 us. At the default
    // 100 kHz it would be ~1.4 ms — a quarter of a PWM period, every 20 ms.
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    // NOTE: deliberately NOT calling Wire.setWireTimeout() here.
    //
    // It looks like cheap insurance against a wedged bus, but measured on this
    // rig it BREAKS the IMU: with a timeout armed, the 14-byte burst read fails
    // essentially every time (gyro calibration collected <50% of its samples
    // and imuTick published nothing at all), while the identical code with no
    // timeout scores 19/20. The `Z` diagnostic only appeared healthy because
    // it re-calls Wire.begin(), which clears the timeout setting.
    //
    // A hung bus is the rarer and more visible failure — and mpuReadRaw already
    // returns false on any error, so a genuinely dead bus degrades to "no IMU"
    // rather than corrupt data.
    imuInit();
    ina219Init();
    lcdInit();

    SPI.begin();
    rfidProbe(true);

    if (lcd_present) lcdSetRow(3, "Calibrating gyro...");
    gyroCalibrate();       // robot must be stationary at power-up
    if (lcd_present) lcdSetRow(3, imu_addr ? "Ready" : "Ready (no IMU)");

    Serial.print(F("S,READY,"));
    Serial.println(F(FW_VERSION));
    last_cmd_ms = millis();
    imu_next_ms = millis();
    sonar_next_ms = millis();

    // Publish the starting door and compartment state so a subscriber that
    // connects later is not left guessing until something physically changes.
    ir_state = ir_candidate = (digitalRead(IR_PIN) == IR_OCCUPIED_LEVEL);
    ir_stable_ms = millis();
    irReport();
    doorReport();
}

void loop() {
    readSerial();
    spwmTick();
    imuTick();
    spwmTick();
    sonarTick();
    spwmTick();
    ina219Tick();
    spwmTick();
    doorTick();
    doorReleaseTick();
    irTick();
    spwmTick();
    rfidTick();
    spwmTick();
    lcdRefresh();
    lcdTick();
    spwmTick();

    // Loop rate, sampled once a second. Cheap: one increment per pass.
    loop_count++;
    if ((long)(millis() - loop_rate_ms) >= 0) {
        loop_rate_hz = loop_count;
        loop_count = 0;
        loop_rate_ms = millis() + 1000UL;
    }

    // Watchdog: stop if the host goes quiet. Raw M/W/Y commands latch for
    // bench testing and are exempt — send S (or reset) to clear.
    if (!raw_mode && (millis() - last_cmd_ms) > CMD_TIMEOUT_MS) {
        stopAll();
        last_cmd_ms = millis();   // re-arm so we brake once, not every pass
    }
}
