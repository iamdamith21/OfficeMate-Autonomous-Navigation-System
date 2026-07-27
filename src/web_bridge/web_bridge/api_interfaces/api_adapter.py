#!/usr/bin/env python3
"""api_adapter — translates the robot's ROS topics into the web app's contract.

WHY THIS NODE EXISTS
--------------------
The robot and the web app were written against different vocabularies. Every
topic the web app subscribed to differed from the robot's in BOTH name and
message type:

    web app wants                     robot publishes
    ─────────────────────────────     ──────────────────────────────────────
    /battery_level      Float32   <-  /battery/state     BatteryState (0..1)
    /nav/status         String    <-  /mission_state     MissionState (uint8)
    /ultrasonic/distance Float32  <-  /ultrasonic/range  Range (metres)
    /locker/status      Bool      <-  /doors/state       String

Rather than push ROS's vocabulary into the browser, this node does the
translation on the robot. That keeps custom message definitions (MissionState)
off the websocket, keeps the payloads tiny, and means the web app needs no
knowledge of the robot's internals. The web app previously only ever worked
against scripts/mock_ros.cjs, which faked exactly these four topics — so this
node is what makes the real robot a drop-in replacement for the mock.

UNITS MATTER — the web app renders these directly:
  * battery is a PERCENTAGE 0..100 (BatteryState.percentage is a 0..1 fraction)
  * obstacle distance is in CENTIMETRES (Range.range is in metres)
Getting either wrong produces a plausible-looking but wrong dashboard.

LIVENESS
--------
The web app decides the robot is "online" if ANY of these topics arrived in the
last 6 s. The ultrasonic stream (~12 Hz, straight off the Arduino) is the
honest heartbeat: it only flows when the board is actually connected and
running. /nav/status is republished at 1 Hz so the status line stays fresh even
when no mission is active.

Deliberately NOT faked: /battery_level. No INA219 is fitted on this rig, so no
BatteryState is ever published, and inventing a number here would put a
confident, wrong battery percentage in front of a user. The web app has been
changed to show "N/A" until a real reading arrives.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, \
    QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState, Range
from std_msgs.msg import Bool, Float32, String

try:
    from robot_interfaces.msg import MissionState
    HAVE_MISSION_STATE = True
except ImportError:      # robot_interfaces not built — degrade, do not crash
    HAVE_MISSION_STATE = False

STATUS_REPUBLISH_HZ = 1.0

# MissionState.state -> the wording the dashboard shows. The web app treats
# this as free display text, so these are written to read naturally next to its
# own DELIVERY_STATES labels rather than to match them exactly.
MISSION_STATE_TEXT = {
    0:   'Initializing',
    1:   'Idle',
    2:   'Mission Received',
    3:   'Planning Route',
    4:   'Navigating',
    5:   'Arrived',
    6:   'Verifying RFID',
    7:   'Opening Compartment',
    8:   'Awaiting Pickup',
    9:   'Closing Compartment',
    10:  'Returning to Base',
    11:  'Delivery Complete',
    255: 'Error',
}


class ApiAdapter(Node):
    def __init__(self):
        super().__init__('api_adapter')

        # Sensor streams are best-effort on the robot side (SensorDataQoS), so
        # subscribing reliably would silently match nothing. This is the same
        # trap that cost us the LiDAR scans earlier in the project.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Latch the status so a browser that connects mid-mission gets the
        # current state immediately instead of waiting for the next tick.
        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_battery = self.create_publisher(Float32, '/battery_level', latched)
        self.pub_status = self.create_publisher(String, '/nav/status', latched)
        self.pub_distance = self.create_publisher(Float32, '/ultrasonic/distance', 10)
        self.pub_locker = self.create_publisher(Bool, '/locker/status', latched)

        self.create_subscription(BatteryState, '/battery/state',
                                 self._battery_cb, 5)
        self.create_subscription(Range, '/ultrasonic/range',
                                 self._range_cb, sensor_qos)
        self.create_subscription(String, '/doors/state', self._doors_cb, 5)

        self._status = 'Idle'
        if HAVE_MISSION_STATE:
            self.create_subscription(MissionState, '/mission_state',
                                     self._mission_cb, 10)
        else:
            self.get_logger().warn(
                'robot_interfaces not importable — /mission_state will not be '
                'translated and the dashboard will show a static "Idle"')

        self.create_timer(1.0 / STATUS_REPUBLISH_HZ, self._republish_status)

        self.get_logger().info(
            'api_adapter up: /battery/state -> /battery_level, '
            '/mission_state -> /nav/status, /ultrasonic/range -> '
            '/ultrasonic/distance (cm), /doors/state -> /locker/status')

    def _battery_cb(self, msg: BatteryState):
        # BatteryState.percentage is documented as a 0..1 fraction, but plenty
        # of drivers fill it with 0..100 already. Detect rather than assume:
        # a value above 1.0 cannot be a fraction.
        pct = msg.percentage
        if pct <= 1.0:
            pct *= 100.0
        self.pub_battery.publish(Float32(data=float(max(0.0, min(100.0, pct)))))

    def _range_cb(self, msg: Range):
        # metres -> centimetres, which is what the dashboard prints.
        self.pub_distance.publish(Float32(data=float(msg.range * 100.0)))

    def _doors_cb(self, msg: String):
        # Web app semantics: true = unlocked/open. The robot reports
        # MOVING | OPEN | CLOSED; treat anything not fully CLOSED as unlocked,
        # which is the safe reading for a "is it secure?" indicator.
        self.pub_locker.publish(Bool(data=msg.data.strip().upper() != 'CLOSED'))

    def _mission_cb(self, msg: MissionState):
        # Prefer the FSM's own human-readable name when it set one.
        self._status = msg.state_name or MISSION_STATE_TEXT.get(
            msg.state, f'State {msg.state}')
        self.pub_status.publish(String(data=self._status))

    def _republish_status(self):
        self.pub_status.publish(String(data=self._status))


def main():
    rclpy.init()
    node = ApiAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
