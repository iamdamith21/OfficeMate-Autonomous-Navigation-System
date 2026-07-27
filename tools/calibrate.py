#!/usr/bin/env python3
"""
calibrate.py — measure ACTUAL robot velocity against COMMANDED velocity.

Why this matters for Nav2
-------------------------
Nav2 assumes that when it asks for 0.25 m/s it gets roughly 0.25 m/s. Three
things break when that is untrue:

  * SimpleProgressChecker aborts a goal if the robot fails to move
    required_movement_radius within movement_time_allowance.
  * RegulatedPurePursuitController computes lookahead from velocity, so a
    wrong scale puts the carrot in the wrong place and the path oscillates.
  * The velocity smoother's accel limits are expressed in real units.

The 1 m goto run took 14.5 s against a commanded 0.20 m/s — roughly 3x slower
than asked. This measures the real curve so MAX_SPEED_MPS in the firmware can
be corrected.

Method
------
Ground truth comes from /odometry/filtered, which is derived from rf2o laser
scan matching — i.e. measured from the environment, independent of the motors.
For each test point: ramp up, hold, measure displacement over the HOLD window
only (ramp excluded), stop, report actual/commanded.

Usage
-----
    python3 calibrate.py                 # linear sweep, then angular
    python3 calibrate.py --linear-only
    python3 calibrate.py --hold 4.0      # longer hold = better signal/noise

Needs several metres of clear straight space. Supervise it.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

PUB_HZ = 20.0
RAMP_S = 1.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Cal:
    def __init__(self):
        self.node = Node('officemate_calibrate')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.x = self.y = self.yaw = None
        self.node.create_subscription(
            Odometry, '/odometry/filtered', self._cb, 10)

    def _cb(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = yaw_of(m.pose.pose.orientation)

    def spin(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def send(self, lin, ang):
        m = Twist()
        m.linear.x = float(lin)
        m.angular.z = float(ang)
        self.pub.publish(m)

    def hold(self, lin, ang, secs):
        """Publish at PUB_HZ for secs, spinning so odometry stays fresh."""
        end = time.monotonic() + secs
        while time.monotonic() < end and rclpy.ok():
            self.send(lin, ang)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(1.0 / PUB_HZ)

    def stop(self):
        for _ in range(10):
            self.send(0.0, 0.0)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(0.02)
        self.spin(0.5)

    def wait_odom(self, secs=10.0):
        end = time.monotonic() + secs
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.x is not None:
                return True
        return False


def measure_linear(cal, cmd, hold_s):
    cal.hold(cmd, 0.0, RAMP_S)                 # ramp/settle, not measured
    x0, y0 = cal.x, cal.y
    t0 = time.monotonic()
    cal.hold(cmd, 0.0, hold_s)
    dt = time.monotonic() - t0
    dist = math.hypot(cal.x - x0, cal.y - y0)
    cal.stop()
    actual = dist / dt if dt > 0 else 0.0
    ratio = actual / cmd if cmd else 0.0
    print(f'  cmd={cmd:5.2f} m/s -> actual={actual:5.3f} m/s  '
          f'({dist:.3f} m in {dt:.1f}s)  ratio={ratio:.2f}')
    return cmd, actual


def measure_angular(cal, cmd, hold_s):
    cal.hold(0.0, cmd, RAMP_S)
    y0 = cal.yaw
    t0 = time.monotonic()
    # accumulate yaw so we survive wrap-around during a long spin
    total = 0.0
    prev = y0
    end = time.monotonic() + hold_s
    while time.monotonic() < end and rclpy.ok():
        cal.send(0.0, cmd)
        rclpy.spin_once(cal.node, timeout_sec=0.0)
        if cal.yaw is not None:
            total += wrap(cal.yaw - prev)
            prev = cal.yaw
        time.sleep(1.0 / PUB_HZ)
    dt = time.monotonic() - t0
    cal.stop()
    actual = abs(total) / dt if dt > 0 else 0.0
    ratio = actual / abs(cmd) if cmd else 0.0
    print(f'  cmd={cmd:5.2f} rad/s -> actual={actual:5.3f} rad/s  '
          f'({math.degrees(abs(total)):.0f}deg in {dt:.1f}s)  ratio={ratio:.2f}')
    return abs(cmd), actual


def fit_scale(pairs):
    """Least-squares slope through the origin: actual = k * commanded."""
    num = sum(c * a for c, a in pairs)
    den = sum(c * c for c, _ in pairs)
    return num / den if den else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hold', type=float, default=3.0)
    ap.add_argument('--linear-only', action='store_true')
    ap.add_argument('--angular-only', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    cal = Cal()
    try:
        if not cal.wait_odom():
            print('ERROR: no /odometry/filtered. Is the stack running?')
            return

        lin_pairs, ang_pairs = [], []

        if not args.angular_only:
            print('\n=== LINEAR ===  (needs ~3 m of clear space ahead)')
            for cmd in (0.10, 0.15, 0.22, 0.30):
                lin_pairs.append(measure_linear(cal, cmd, args.hold))
                cal.spin(1.0)

        if not args.linear_only:
            print('\n=== ANGULAR ===  (spins in place)')
            for cmd in (0.5, 0.9, 1.4):
                ang_pairs.append(measure_angular(cal, cmd, args.hold))
                cal.spin(1.0)

        print('\n=== RESULT ===')
        if lin_pairs:
            k = fit_scale(lin_pairs)
            print(f'linear  : actual = {k:.3f} x commanded')
            if k > 0:
                print(f'          -> firmware MAX_SPEED_MPS should be scaled by {k:.3f}')
                print(f'          -> current 0.681 m/s becomes {0.681 * k:.3f} m/s')
        if ang_pairs:
            ka = fit_scale(ang_pairs)
            print(f'angular : actual = {ka:.3f} x commanded')
            if ka > 0:
                print(f'          -> firmware TURN_GAIN 2.0 becomes {2.0 / ka:.2f}')
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        try:
            cal.stop()
            cal.node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
