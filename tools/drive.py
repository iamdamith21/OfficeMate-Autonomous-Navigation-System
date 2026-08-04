#!/usr/bin/env python3
"""
drive.py — continuous-publish teleop for OfficeMate mapping.

Why this exists instead of teleop_twist_keyboard
------------------------------------------------
teleop_twist_keyboard's getKey() calls sys.stdin.read(1), which BLOCKS. It
publishes exactly ONE Twist per keypress and then waits. It relies entirely on
the subscriber latching that single message — so over wifi, one dropped
datagram means the command silently never happens, with nothing to retry
against. That is why key presses did nothing while `ros2 topic pub -r 10`
worked: the latter sent dozens of messages.

This node republishes the current command at 20 Hz, so a lost packet costs
50 ms instead of the whole command.

It also ramps acceleration rather than stepping instantly. Instant velocity
steps are a major cause of scan-matching failure: rf2o estimates motion by
matching consecutive scans, and a sudden jump exceeds what it can track,
injecting odometry error that shows up as smeared walls.

Run it ON THE PI if you can — then /cmd_vel never crosses wifi at all.

Usage
-----
    python3 drive.py                # interactive (needs a focused terminal)
    python3 drive.py --selftest 3   # no keyboard: ramp forward 3 s, stop, exit

Keys
----
    w : forward        s : reverse        x : straighten (cancel turn only)
    a : bank left      d : bank right     space : STOP
    q : spin left      e : spin right     (in place, no forward motion)
    + / - : max linear speed              [ / ] : max turn rate
    z : quit (sends stop first)

  a/d keep whatever forward speed you already have, so w-then-a gives a smooth
  ARC. Use q/e only when you genuinely want to pivot on the spot — spinning
  scrubs all four wheels and is the worst case for scan matching.
"""
import argparse
import math
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    """Yaw (rad) from a quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))

PUB_HZ = 20.0

# Ramp rates. Deliberately gentle: rf2o estimates motion by matching successive
# scans, and a step change in velocity exceeds what it can track, injecting the
# odometry error that smears walls. Angular is the slower of the two because
# rotation is what scan matching handles worst.
ACCEL = 0.15        # m/s² (gentle ramp for scan-matching)
ANG_ACCEL = 0.5     # rad/s² (gentle turn ramp)

# Defaults matched to the MEASURED limits (2026-07-27 floor calibration), not
# to what feels responsive. Both of these were previously above the calibrated
# ceiling, which actively degrades a map:
#
#   Linear 0.22 was above the 0.20 m/s ceiling. 0.15 tracked at ratio 1.00
#   while 0.22 and 0.30 were erratic, so 0.18 is the honest cruise.
#
#   Angular 0.9 was the worse offender. The firmware's TURN_GAIN_PIVOT (6.5)
#   is sized so 0.50 rad/s maps onto FULL PWM -- the gain must always be
#   MAX_SPEED_MPS / (max_ang * WHEEL_SEP/2). Commanding 0.9 therefore
#   saturates: every turn becomes full deflection regardless of what was
#   asked for, achieved no longer tracks commanded, and rf2o -- which
#   estimates motion by matching successive scans -- cannot follow it. That
#   is the odometry error that smears walls. 0.45 is where the measured
#   commanded/achieved ratio was 1.02.
#
# RAISED 2026-08-02 off the true hardware limits rather than the cautious
# floor-calibration figures above. The real saturation points, from the
# firmware drive model (MIN_PWM 90, pwm = 90 + frac*165, MAX_SPEED_MPS
# = MAX_RPM 100 / 60 * 2*pi * WHEEL_RADIUS 0.065 = 0.681 m/s):
#
#   Linear  saturates at 0.681 m/s.
#   Angular saturates at 0.635 rad/s -- at a pivot, half_track =
#           (WHEEL_SEP/2) * TURN_GAIN_PIVOT = 0.165 * 6.5 = 1.0725, so
#           wheel = ang * 1.0725 hits MAX_SPEED_MPS at ang = 0.635.
#           ABOVE 0.635 NOTHING CHANGES AT THE WHEELS -- the extra command is
#           pure fiction that only lies to odometry. That is what wrecked the
#           earlier maps (sessions were running at 2.5 rad/s, 4x fictional).
#
# So the honest usable band is everything BELOW those saturation points, and
# the previous 0.20/0.50 caps were leaving real, trackable performance unused.
# Defaults also raised because low PWM was stalling the robot outright: at
# 0.18 m/s the wheels only see pwm 134, which on a 5-6 kg chassis with the
# known pack sag is marginal for breaking stiction -- the "gets stuck".
DEF_LIN = 0.20
DEF_ANG = 0.35

# (linear_sign, angular_sign, keep_linear, keep_angular)
# keep_* means "leave that axis alone" — which is what turns an on-the-spot
# spin into a smooth arc: pressing 'a' while rolling keeps the forward speed.
MOVE = {
    'w': (1, 0, False, True),    # forward, keep any existing turn
    's': (-1, 0, False, True),   # reverse, keep any existing turn
    'a': (0, 1, True, False),    # bank left  — keep forward speed
    'd': (0, -1, True, False),   # bank right — keep forward speed
    'x': (0, 0, True, False),    # straighten: cancel turn, keep forward
    'q': (0, 1, False, False),   # spin left in place
    'e': (0, -1, False, False),  # spin right in place
    ' ': (0, 0, False, False),   # full stop
}

HELP = __doc__.split('Keys\n----\n')[1]


def ramp(cur, tgt, step):
    if cur < tgt:
        return min(cur + step, tgt)
    if cur > tgt:
        return max(cur - step, tgt)
    return cur


class Drive:
    """Single-threaded: ROS spin, key reads and publishing share one loop.

    No background spin thread, so there is no race between it and
    rclpy.shutdown() on exit (which produced a std::terminate abort).
    """

    def __init__(self):
        self.node = Node('officemate_drive')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.max_lin = DEF_LIN
        self.max_ang = DEF_ANG
        self.tgt_lin = self.tgt_ang = 0.0
        self.cur_lin = self.cur_ang = 0.0
        self.yaw = None
        self.node.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10)

    def _odom_cb(self, msg):
        self.yaw = yaw_of(msg.pose.pose.orientation)

    def publish_tick(self, dt):
        self.cur_lin = ramp(self.cur_lin, self.tgt_lin, ACCEL * dt)
        self.cur_ang = ramp(self.cur_ang, self.tgt_ang, ANG_ACCEL * dt)
        msg = Twist()
        msg.linear.x = self.cur_lin
        msg.angular.z = self.cur_ang
        self.pub.publish(msg)

    def handle_key(self, key):
        """Return False to quit."""
        if key in MOVE:
            lin_sign, ang_sign, keep_lin, keep_ang = MOVE[key]
            if not keep_lin:
                self.tgt_lin = lin_sign * self.max_lin
            if not keep_ang:
                self.tgt_ang = ang_sign * self.max_ang
        # CEILINGS SET 2026-08-02 to the hardware saturation points, not above
        # them. Originally 0.7 m/s / 2.5 rad/s, which let a held '+' / ']' push
        # the command to 4x the point where the wheels are already flat out --
        # sessions were running at -0.700 / -2.500 and the extra was pure
        # fiction that only corrupted odometry and smeared the map. These caps
        # are the real limits (see the DEF_LIN note): 0.60 rad/s sits just
        # under the 0.635 pivot saturation, so everything up to the cap is
        # still genuinely trackable. Going higher does not make the robot
        # turn faster -- it only makes odometry wrong.
        elif key == '+':
            self.max_lin = min(self.max_lin * 1.1, 0.68)
        elif key == '-':
            self.max_lin = max(self.max_lin * 0.9, 0.05)
        elif key == '[':
            self.max_ang = max(self.max_ang * 0.9, 0.1)
        elif key == ']':
            self.max_ang = min(self.max_ang * 1.1, 0.635)
        elif key in ('z', '\x03'):
            return False
        else:
            self.tgt_lin = self.tgt_ang = 0.0
        return True

    def hard_stop(self):
        """Publish real zeros immediately, bypassing the ramp."""
        self.tgt_lin = self.tgt_ang = 0.0
        self.cur_lin = self.cur_ang = 0.0
        msg = Twist()
        for _ in range(10):
            self.pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(0.02)

    def subscribers(self):
        """How many nodes are listening to /cmd_vel.

        Zero means arduino_bridge is not running, so commands go nowhere and
        the robot sits still with no error anywhere. That is silent by default
        in ROS, so surface it.
        """
        return self.pub.get_subscription_count()

    def status(self):
        n = self.subscribers()
        link = f'sub={n}' if n else 'sub=0 !! NOBODY LISTENING !!'
        return (f'\rcmd lin={self.cur_lin:+.2f} ang={self.cur_ang:+.2f}   '
                f'max lin={self.max_lin:.2f} turn={self.max_ang:.2f}   '
                f'{link}    ')


def run(drive, interactive, selftest_s):
    period = 1.0 / PUB_HZ
    deadline = time.monotonic() + selftest_s if selftest_s > 0 else None
    if selftest_s > 0:
        drive.tgt_lin = drive.max_lin

    last = time.monotonic()
    while rclpy.ok():
        now = time.monotonic()
        dt = max(now - last, 1e-3)
        last = now

        if interactive and select.select([sys.stdin], [], [], 0)[0]:
            if not drive.handle_key(sys.stdin.read(1)):
                break
            sys.stdout.write(drive.status())
            sys.stdout.flush()

        drive.publish_tick(dt)
        rclpy.spin_once(drive.node, timeout_sec=0.0)

        if deadline and time.monotonic() >= deadline:
            break
        time.sleep(period)


def segment(drive, lin, ang, label, secs=None, yaw_deg=None, cap=25.0,
            fallback_secs=None):
    """Hold a commanded velocity until a time or heading-change goal is met.

    Ramping is left to publish_tick, so every segment eases in and out rather
    than stepping. Returns the heading change actually achieved, in degrees.
    """
    drive.tgt_lin = lin
    drive.tgt_ang = ang
    start = time.monotonic()
    yaw0 = drive.yaw
    have_odom = yaw0 is not None
    turned = 0.0
    note = '' if have_odom else '  [NO ODOM — timing fallback]'
    print(f'  {label}: lin={lin:+.2f} ang={ang:+.2f}{note}', flush=True)

    while rclpy.ok():
        drive.publish_tick(1.0 / PUB_HZ)
        rclpy.spin_once(drive.node, timeout_sec=0.0)
        time.sleep(1.0 / PUB_HZ)

        if yaw0 is not None and drive.yaw is not None:
            turned = abs(wrap(drive.yaw - yaw0))

        elapsed = time.monotonic() - start
        if secs is not None and elapsed >= secs:
            break
        if yaw_deg is not None:
            if have_odom and math.degrees(turned) >= yaw_deg:
                break
            # Without odometry we cannot measure the turn, so fall back to a
            # bounded time rather than arcing until the safety cap.
            if not have_odom and fallback_secs and elapsed >= fallback_secs:
                break
        if elapsed >= cap:                      # safety: never run away
            print('    (segment cap hit)')
            break

    print(f'    -> {math.degrees(turned):.1f}deg in {time.monotonic()-start:.1f}s')
    return math.degrees(turned)


def demo_arc(drive):
    """forward -> smooth 90 deg left arc -> forward, as requested."""
    if drive.yaw is None:
        print('  (no /odometry/filtered yet — arc will fall back to timing)')
    print('\n=== ARC DEMO: forward 3s, 90deg left arc, forward 3s ===')
    segment(drive, drive.max_lin, 0.0, 'forward', secs=3.0)
    # ~90 deg at 0.9 rad/s is ~1.75 s of steady turn, plus ramp in and out.
    turned = segment(drive, drive.max_lin * 0.8, drive.max_ang,
                     'arc left 90deg', yaw_deg=90.0, cap=12.0,
                     fallback_secs=3.2)
    segment(drive, drive.max_lin, 0.0, 'forward again', secs=3.0)
    drive.hard_stop()
    print(f'\nARC RESULT: turned {turned:.1f} deg (target 90)')
    return turned


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--selftest', type=float, default=0.0,
                    help='seconds to drive forward without a keyboard, then stop')
    ap.add_argument('--demo-arc', action='store_true',
                    help='forward, smooth 90deg left arc, forward — then stop')
    args = ap.parse_args()

    rclpy.init()
    drive = Drive()
    scripted = args.selftest > 0 or args.demo_arc
    interactive = not scripted and sys.stdin.isatty()
    settings = None

    try:
        if args.selftest > 0:
            print(f'selftest: forward {drive.max_lin} m/s for {args.selftest}s')
        elif args.demo_arc:
            print('arc demo')
        elif not interactive:
            print('stdin is not a TTY — run this in a real terminal, '
                  'or use --selftest N')
            return
        else:
            print(HELP)

        # Give discovery a moment (3.5s), then check somebody is actually listening.
        for _ in range(70):
            rclpy.spin_once(drive.node, timeout_sec=0.05)
            if drive.subscribers():
                break
        if not drive.subscribers():
            print('\n*** WARNING: nothing is subscribed to /cmd_vel. ***')
            print('    arduino_bridge is not running, so the robot will NOT move.')
            print('    Start the stack first:')
            print('      ros2 launch robot_mapping mapping.launch.py arduino_dev:=/dev/ttyACM0\n')
        else:
            print(f'/cmd_vel subscribers: {drive.subscribers()} — good.\n')

        if interactive:
            settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        if args.demo_arc:
            # let odometry arrive before we measure heading against it
            for _ in range(40):
                rclpy.spin_once(drive.node, timeout_sec=0.05)
                if drive.yaw is not None:
                    break
            demo_arc(drive)
        else:
            run(drive, interactive, args.selftest)
    except KeyboardInterrupt:
        pass
    finally:
        if settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            drive.hard_stop()
            drive.node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
        print('\nstopped.')


if __name__ == '__main__':
    main()
