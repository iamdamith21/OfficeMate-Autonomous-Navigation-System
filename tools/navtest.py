#!/usr/bin/env python3
"""
navtest.py — seed AMCL, then send a goal, exactly as the RViz buttons do.

  2D Pose Estimate  ->  /initialpose  (geometry_msgs/PoseWithCovarianceStamped)
  2D Goal Pose      ->  /goal_pose    (geometry_msgs/PoseStamped)

This publishes the same two messages, so anything proven here works when you
click. It then watches /amcl_pose covariance to judge whether localisation
actually converged, rather than assuming it did.

  python3 navtest.py --init 0 0 0            # seed AMCL only, report convergence
  python3 navtest.py --init 0 0 0 --goal 1.0 0 0
  python3 navtest.py --goal 1.0 0 0          # goal only, keep current estimate
"""
import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)


def quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class NavTest(Node):
    def __init__(self):
        super().__init__('navtest')
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        amcl_qos = QoSProfile(depth=5,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              history=HistoryPolicy.KEEP_LAST)
        self.amcl = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._amcl_cb, amcl_qos)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _amcl_cb(self, msg):
        self.amcl = msg

    def spin(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_initial(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = float(x)
        m.pose.pose.position.y = float(y)
        qz, qw = quat(float(yaw))
        m.pose.pose.orientation.z = qz
        m.pose.pose.orientation.w = qw
        # Same covariance RViz's 2D Pose Estimate tool uses.
        m.pose.covariance[0] = 0.25
        m.pose.covariance[7] = 0.25
        m.pose.covariance[35] = 0.06853891909122467
        for _ in range(3):
            self.init_pub.publish(m)
            self.spin(0.2)
        print(f'  published /initialpose  x={x} y={y} yaw={yaw}')

    def report_amcl(self, label):
        if self.amcl is None:
            print(f'  {label}: no /amcl_pose received')
            return None
        p = self.amcl.pose.pose.position
        c = self.amcl.pose.covariance
        sx, sy, syaw = math.sqrt(abs(c[0])), math.sqrt(abs(c[7])), math.sqrt(abs(c[35]))
        print(f'  {label}: x={p.x:+.3f} y={p.y:+.3f}  '
              f'sigma_x={sx:.3f} sigma_y={sy:.3f} sigma_yaw={math.degrees(syaw):.1f}deg')
        return sx, sy, syaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--init', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args()

    rclpy.init()
    n = NavTest()
    try:
        n.spin(2.0)

        if args.init:
            print('--- 2D Pose Estimate ---')
            n.set_initial(*args.init)
            n.spin(3.0)
            n.report_amcl('after seeding')
            print('  settling (AMCL refines against /scan)...')
            n.spin(5.0)
            n.report_amcl('after settle')

        if not args.goal:
            return

        print('\n--- 2D Goal Pose ---')
        if not n.nav.wait_for_server(timeout_sec=10.0):
            print('  ERROR: navigate_to_pose action server unavailable')
            return

        gx, gy, gyaw = args.goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = n.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        qz, qw = quat(gyaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        print(f'  sending goal x={gx} y={gy} yaw={gyaw}')

        fut = n.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('  GOAL REJECTED')
            return
        print('  goal accepted, navigating...')

        res_fut = gh.get_result_async()
        start = time.monotonic()
        last = 0.0
        while rclpy.ok() and not res_fut.done():
            rclpy.spin_once(n, timeout_sec=0.2)
            el = time.monotonic() - start
            if el - last >= 5.0:
                last = el
                n.report_amcl(f'  t={el:5.1f}s')
            if el > args.timeout:
                print('  TIMEOUT — cancelling')
                gh.cancel_goal_async()
                n.spin(3.0)
                break

        if res_fut.done():
            st = res_fut.result().status
            names = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                     GoalStatus.STATUS_ABORTED: 'ABORTED',
                     GoalStatus.STATUS_CANCELED: 'CANCELED'}
            print(f'\n  RESULT: {names.get(st, st)} after {time.monotonic()-start:.1f}s')
            n.report_amcl('final pose')
            if n.amcl:
                p = n.amcl.pose.pose.position
                err = math.hypot(p.x - gx, p.y - gy)
                print(f'  distance from requested goal: {err:.3f} m')
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        try:
            n.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
