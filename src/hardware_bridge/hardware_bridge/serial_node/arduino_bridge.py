#!/usr/bin/env python3
"""
arduino_bridge — bidirectional serial bridge to the Arduino Mega hub.

Peer firmware: hardware_bridge/arduino/robot_firmware/robot_firmware.ino
(protocol documented in its header). Replaces the old one-way
cmd_vel_serial_bridge.py.

ROS interface
  subs : /cmd_vel               geometry_msgs/Twist   (resent at 20 Hz, watchdog)
         /lcd/line2             std_msgs/String       → LCD second line
  pubs : /imu/data_raw          sensor_msgs/Imu       50 Hz, frame imu_link
         /ultrasonic/range      sensor_msgs/Range     ~15 Hz, frame ultrasonic_link
         /battery/state         sensor_msgs/BatteryState  1 Hz
         /rfid/tag              std_msgs/String       on scan
         /compartment/occupied  std_msgs/Bool         on change + 2 s refresh
         /doors/state           std_msgs/String       MOVING | OPEN | CLOSED
  srvs : /doors/open, /doors/close   std_srvs/Trigger (waits for firmware ack)

Safety: if the last ultrasonic reading is closer than safety_stop_distance,
forward motion is clamped to zero before it reaches the motors (reverse and
rotation stay allowed). This catches low obstacles the lidar cannot see even
if Nav2's costmap misses them. Stale (>1 s) readings do not clamp, so an
unplugged sensor cannot immobilise the robot.

The serial link is owned by a reader thread with a reconnect loop: the node
keeps running (and bringup stays healthy) when the board is unplugged.
"""
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, Range
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

CMD_RATE_HZ = 20.0
RECONNECT_S = 3.0
DOOR_ACK_TIMEOUT_S = 4.0

# Opening the port toggles DTR, which resets the Mega into its bootloader. The
# bootloader only hands over to the sketch after the line goes quiet — and this
# node's 20 Hz cmd timer starts writing immediately, so without this wait the
# bootloader is held hostage forever and the sketch NEVER runs. Symptom: the
# port opens fine, no 'S,READY' banner ever arrives, and every drive command is
# silently ignored. Must exceed the Mega bootloader timeout (~1 s).
#
# Raised to 6 s for firmware v10: the sketch now averages the gyro for ~2 s at
# boot to measure its bias, and the robot must be stationary and un-commanded
# for the whole of it. Staying quiet until that finishes also keeps the Mega's
# 64-byte serial RX buffer from overflowing while the sketch is not reading it.
BOOT_WAIT_S = 6.0

# Stop if /cmd_vel goes silent. Without this the last commanded velocity is
# resent at CMD_RATE_HZ forever, so a teleop that crashes or is killed leaves
# the robot driving with nothing left to stop it — the firmware watchdog cannot
# help, because this node is still faithfully sending the stale value.
CMD_TIMEOUT_S = 0.5

# 3S li-ion pack voltage window for the percentage estimate
BATT_EMPTY_V = 9.0
BATT_FULL_V = 12.6

SONAR_MIN_M = 0.02
SONAR_MAX_M = 3.0  # MUST match SONAR_MAX_M in robot_firmware.ino
SONAR_FOV_RAD = 0.26  # ~15°


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('safety_stop_distance', 0.15)  # 0 disables
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('ultrasonic_frame', 'ultrasonic_link')

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.stop_dist = self.get_parameter('safety_stop_distance').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.sonar_frame = self.get_parameter('ultrasonic_frame').value

        self.ser = None
        self.ser_lock = threading.Lock()

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_cmd_time = 0.0
        self.last_range = None
        self.last_range_time = 0.0

        self.door_state = 'CLOSED'
        self.door_event = threading.Event()

        # pubs
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 50)
        self.pub_range = self.create_publisher(Range, '/ultrasonic/range', 10)
        self.pub_batt = self.create_publisher(BatteryState, '/battery/state', 5)
        self.pub_rfid = self.create_publisher(String, '/rfid/tag', 5)
        self.pub_ir = self.create_publisher(Bool, '/compartment/occupied', 5)
        self.pub_doors = self.create_publisher(String, '/doors/state', 5)

        # subs + cmd timer share the default group; door services get their
        # own so their blocking ack-wait cannot stall the cmd_vel stream.
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(String, '/lcd/line2', self._lcd_cb, 5)
        self.create_timer(1.0 / CMD_RATE_HZ, self._send_cmd)

        srv_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, '/doors/open',
                            lambda rq, rs: self._door_srv(rs, True),
                            callback_group=srv_group)
        self.create_service(Trigger, '/doors/close',
                            lambda rq, rs: self._door_srv(rs, False),
                            callback_group=srv_group)

        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()

    # ── outgoing ────────────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        self.last_cmd_time = time.monotonic()

    def _send_cmd(self):
        # Dead-man: a publisher that dies must not leave us driving.
        if (self.linear_x or self.angular_z) and \
                time.monotonic() - self.last_cmd_time > CMD_TIMEOUT_S:
            self.get_logger().warn(
                f'/cmd_vel silent for >{CMD_TIMEOUT_S}s — stopping',
                throttle_duration_sec=5.0)
            self.linear_x = 0.0
            self.angular_z = 0.0

        lin = self.linear_x
        if (self.stop_dist > 0 and lin > 0
                and self.last_range is not None
                and self.last_range < self.stop_dist
                and time.monotonic() - self.last_range_time < 1.0):
            lin = 0.0
        self._write_line(f'V,{lin:.3f},{self.angular_z:.3f}')

    def _lcd_cb(self, msg: String):
        self._write_line('L,' + msg.data[:20].replace('\n', ' '))

    def _door_srv(self, resp, open_doors: bool):
        target = 'OPEN' if open_doors else 'CLOSED'
        self.door_event.clear()
        if not self._write_line('O' if open_doors else 'C'):
            resp.success = False
            resp.message = 'Arduino not connected'
            return resp
        deadline = time.monotonic() + DOOR_ACK_TIMEOUT_S
        while time.monotonic() < deadline:
            if self.door_event.wait(timeout=0.1) and self.door_state == target:
                resp.success = True
                resp.message = f'Doors {target.lower()}'
                return resp
            self.door_event.clear()
        resp.success = False
        resp.message = f'No {target} ack within {DOOR_ACK_TIMEOUT_S}s'
        return resp

    def _write_line(self, line: str) -> bool:
        with self.ser_lock:
            if self.ser is None:
                # Throttled, or this fires 20x/s while the board is absent.
                self.get_logger().warn(
                    'Serial not ready — drive commands are being dropped',
                    throttle_duration_sec=5.0)
                return False
            try:
                self.ser.write((line + '\n').encode('ascii'))
                return True
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f'Serial write failed: {e}',
                                       throttle_duration_sec=5.0)
                return False  # reader loop handles the reconnect

    # ── incoming ────────────────────────────────────────────────────────────
    def _reader_loop(self):
        while rclpy.ok():
            if self.ser is None:
                try:
                    ser = serial.Serial(self.port, self.baud, timeout=1.0)
                    # Let the bootloader time out and start the sketch BEFORE
                    # publishing self.ser — _write_line returns False while it
                    # is None, so the cmd timer stays quiet during the wait.
                    self.get_logger().info(
                        f'Opened {self.port} @ {self.baud}; '
                        f'waiting {BOOT_WAIT_S}s for the sketch to boot')
                    time.sleep(BOOT_WAIT_S)
                    try:
                        ser.reset_input_buffer()
                    except (serial.SerialException, OSError):
                        pass
                    with self.ser_lock:
                        self.ser = ser
                    self.get_logger().info('Serial ready')
                except (serial.SerialException, OSError) as e:
                    self.get_logger().warn(
                        f'Serial open failed ({e}); retrying in {RECONNECT_S}s',
                        throttle_duration_sec=30.0)
                    time.sleep(RECONNECT_S)
                    continue
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial lost ({e}); reconnecting')
                with self.ser_lock:
                    try:
                        self.ser.close()
                    except (serial.SerialException, OSError):
                        pass
                    self.ser = None
                continue
            line = raw.decode('ascii', errors='replace').strip()
            if line:
                try:
                    self._handle_line(line)
                except (ValueError, IndexError):
                    self.get_logger().warn(f'Bad line: {line!r}',
                                           throttle_duration_sec=10.0)

    def _handle_line(self, line: str):
        tag, _, rest = line.partition(',')
        if tag == 'I':
            self._publish_imu(rest)
        elif tag == 'U':
            self._publish_range(float(rest))
        elif tag == 'B':
            self._publish_battery(rest)
        elif tag == 'R':
            self.pub_rfid.publish(String(data=rest.upper()))
            self.get_logger().info(f'RFID tag scanned: {rest.upper()}')
        elif tag == 'D':
            self.pub_ir.publish(Bool(data=rest.strip() == '1'))
        elif tag == 'A':
            parts = rest.split(',')
            if len(parts) == 2 and parts[0] == 'DOORS':
                self.door_state = parts[1]
                self.pub_doors.publish(String(data=parts[1]))
                self.door_event.set()
        elif tag == 'S':
            self.get_logger().info(f'Firmware ready: {rest}')
        elif tag == 'E':
            self.get_logger().warn(f'Firmware error: {rest}')

    def _publish_imu(self, csv: str):
        ax, ay, az, gx, gy, gz = (float(x) for x in csv.split(','))
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.orientation_covariance[0] = -1.0  # no orientation estimate
        for i in (0, 4, 8):
            # Measured at rest on the real MPU6500 (sensor_check.py): gyro
            # noise sd was 0.0025 rad/s on all three axes, and the boot-time
            # bias calibration leaves a residual of ~0.0001 rad/s. 0.005 rad/s
            # is that measurement with 2x headroom for vibration while driving.
            # Do not tighten it further without re-measuring under load — an
            # over-confident gyro will fight rf2o instead of complementing it.
            msg.angular_velocity_covariance[i] = 2.5e-5     # (0.005 rad/s)²
            # Accel is NOT fused (see ekf.yaml imu0_config) — this rig's part
            # reads ~10.32 m/s² for gravity, a ~5% scale error typical of these
            # clones. Left deliberately loose; fix by calibrating the part
            # before ever enabling accel fusion.
            msg.linear_acceleration_covariance[i] = 0.25    # (0.5 m/s²)²
        self.pub_imu.publish(msg)

    def _publish_range(self, dist: float):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.sonar_frame
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = SONAR_FOV_RAD
        msg.min_range = SONAR_MIN_M
        msg.max_range = SONAR_MAX_M
        # no echo (-1) → report max_range so the costmap range layer clears
        msg.range = SONAR_MAX_M if dist < 0 else min(dist, SONAR_MAX_M)
        self.pub_range.publish(msg)
        self.last_range = msg.range
        self.last_range_time = time.monotonic()

    def _publish_battery(self, csv: str):
        volts, amps = (float(x) for x in csv.split(','))
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = volts
        msg.current = -amps  # BatteryState: negative while discharging
        pct = (volts - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V)
        msg.percentage = max(0.0, min(1.0, pct))
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.pub_batt.publish(msg)


def main():
    rclpy.init()
    node = ArduinoBridge()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # stop the motors on the way out; skip ROS teardown niceties —
        # a second SIGINT during destroy_node just prints a traceback
        with node.ser_lock:
            ser, node.ser = node.ser, None
            if ser is not None:
                try:
                    ser.write(b'V,0.000,0.000\n')
                    ser.close()
                except (serial.SerialException, OSError):
                    pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
