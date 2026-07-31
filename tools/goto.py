#!/usr/bin/env python3
"""
goto.py — drive to a pose using closed-loop feedback from the live TF.

Scope, stated plainly
---------------------
This is NOT navigation. There is no planner, no costmap and no obstacle
avoidance — this build has no ultrasonic fitted and the LiDAR is not consulted
here. It drives a straight line to the target and will happily drive into
anything in the way. Use it for short, supervised hops across ground you can
see is clear. For real autonomy, use Nav2 (robot_navigation).

It closes the loop on `map -> base_footprint`, which slam_toolbox publishes
while the mapping stack runs, so it corrects for wheel slip and skid — unlike
an open-loop "drive forward for N seconds".

Usage
-----
    python3 goto.py --forward 1.0          # 1 m along the current heading
    python3 goto.py --x 1.5 --y 0.3        # absolute point in the map frame
    python3 goto.py --forward 1.0 --dry    # print the plan, drive nothing

Safety
------
  --max-time  hard timeout, always applied
  --max-dist  abort if we travel further than this (runaway guard)
  Ctrl-C stops the robot before exiting.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

PUB_HZ = 20.0
ACCEL = 0.30
ANG_ACCEL = 0.9

CRUISE_LIN = 0.20         # m/s while approaching
CRUISE_ANG = 0.7          # rad/s while turning on the spot
POS_TOL = 0.08            # m — close enough
YAW_TOL = math.radians(8)  # heading error we accept before driving forward
K_ANG = 1.5               # proportional steering gain while driving


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def ramp(cur, tgt, step):
    if cur < tgt:
        return min(cur + step, tgt)
    if cur > tgt:
        return max(cur - step, tgt)
    return cur


class GoTo:
    def __init__(self, map_frame, base_frame):
        self.node = Node('officemate_goto')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self.node)
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.cur_lin = self.cur_ang = 0.0

    def pose(self):
        """Current (x, y, yaw) in the map frame, or None if TF is not ready."""
        try:
            t = self.buf.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_of(t.transform.rotation)

    def send(self, lin, ang, dt):
        self.cur_lin = ramp(self.cur_lin, lin, ACCEL * dt)
        self.cur_ang = ramp(self.cur_ang, ang, ANG_ACCEL * dt)
        msg = Twist()
        msg.linear.x = self.cur_lin
        msg.angular.z = self.cur_ang
        self.pub.publish(msg)

    def stop(self):
        self.cur_lin = self.cur_ang = 0.0
        msg = Twist()
        for _ in range(10):
            self.pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(0.02)

    def wait_for_tf(self, secs=10.0):
        end = time.time() + secs
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.pose() is not None:
                return True
        return False


def run(g, tx, ty, max_time, max_dist):
    period = 1.0 / PUB_HZ
    start = time.monotonic()
    p0 = g.pose()
    sx, sy, _ = p0
    last = time.monotonic()
    phase = 'turn'

    while rclpy.ok():
        rclpy.spin_once(g.node, timeout_sec=0.0)
        now = time.monotonic()
        dt = max(now - last, 1e-3)
        last = now

        p = g.pose()
        if p is None:
            g.send(0.0, 0.0, dt)
            time.sleep(period)
            continue
        x, y, yaw = p

        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        yaw_err = wrap(bearing - yaw)

        travelled = math.hypot(x - sx, y - sy)
        elapsed = now - start

        if dist <= POS_TOL:
            print(f'\nARRIVED: {dist:.3f} m from target after {elapsed:.1f}s')
            break
        if elapsed > max_time:
            print(f'\nABORT: timeout after {elapsed:.1f}s, still {dist:.2f} m away')
            break
        if travelled > max_dist:
            print(f'\nABORT: travelled {travelled:.2f} m (limit {max_dist})')
            break

        if phase == 'turn':
            # Point at the target before moving, so the approach is a straight
            # line rather than a long curve.
            if abs(yaw_err) <= YAW_TOL:
                phase = 'drive'
            else:
                g.send(0.0, math.copysign(CRUISE_ANG, yaw_err), dt)
        if phase == 'drive':
            # Ease off as we arrive so we do not overshoot and hunt.
            lin = min(CRUISE_LIN, max(0.08, dist * 0.8))
            g.send(lin, max(-CRUISE_ANG, min(CRUISE_ANG, K_ANG * yaw_err)), dt)

        # Throttle the status line: at 20 Hz this floods any log it lands in.
        if int(elapsed * 2) != int((elapsed - dt) * 2):
            print(f'  t={elapsed:4.1f}s pos=({x:+.2f},{y:+.2f}) '
                  f'yaw={math.degrees(yaw):+6.1f}deg dist={dist:.2f}m '
                  f'yaw_err={math.degrees(yaw_err):+6.1f}deg [{phase}]',
                  flush=True)
        time.sleep(period)

    g.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, help='target X in the map frame')
    ap.add_argument('--y', type=float, help='target Y in the map frame')
    ap.add_argument('--forward', type=float,
                    help='metres straight ahead of the current heading')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='base_footprint')
    ap.add_argument('--max-time', type=float, default=40.0)
    ap.add_argument('--max-dist', type=float, default=3.0)
    ap.add_argument('--dry', action='store_true',
                    help='print the plan and exit without moving')
    args = ap.parse_args()

    rclpy.init()
    g = GoTo(args.map_frame, args.base_frame)
    try:
        if not g.wait_for_tf():
            print(f'ERROR: no {args.map_frame} -> {args.base_frame} transform. '
                  'Is the mapping stack running?')
            return
        x, y, yaw = g.pose()
        print(f'current pose: x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+.1f}deg')

        if args.forward is not None:
            tx = x + args.forward * math.cos(yaw)
            ty = y + args.forward * math.sin(yaw)
        elif args.x is not None and args.y is not None:
            tx, ty = args.x, args.y
        else:
            print('give either --forward N or both --x and --y')
            return

        print(f'target pose:  x={tx:+.3f} y={ty:+.3f}  '
              f'(distance {math.hypot(tx-x, ty-y):.2f} m)')
        if args.dry:
            print('dry run — not moving')
            return

        run(g, tx, ty, args.max_time, args.max_dist)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        try:
            g.stop()
            g.node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
