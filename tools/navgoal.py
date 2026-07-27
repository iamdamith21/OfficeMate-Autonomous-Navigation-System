#!/usr/bin/env python3
"""Send a real Nav2 NavigateToPose goal and report what actually happened.

Unlike goto.py this goes through the full stack — planner, costmaps, controller,
recoveries — so it exercises the retuned angular limits under Nav2's own
control rather than under a hand-written velocity loop.

Reports, once a second: distance to goal, the robot's heading, and the angular
velocity actually being achieved (from the fused odometry). The angular column
is the interesting one: with the old 0.20 rad/s cap every turn pinned at the
limit, so seeing intermediate values is the evidence that the proportional band
is real under Nav2 too.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class NavGoal(Node):
    def __init__(self):
        super().__init__('nav_goal_test')
        self.cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(Odometry, '/odometry/filtered', self._odom, 10)
        self.create_subscription(Path, '/plan', self._plan, 5)
        # Pose must come from TF (map frame). /odometry/filtered is in the ODOM
        # frame, so comparing it against a map-frame goal silently reports a
        # meaningless distance -- which is exactly what happened on the first
        # run of this script.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.vyaw = 0.0
        self.plan_len = 0
        self.max_vyaw = 0.0

    def _odom(self, m):
        self.vyaw = m.twist.twist.angular.z
        self.max_vyaw = max(self.max_vyaw, abs(self.vyaw))

    @property
    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()).transform
            class P:
                pass
            p = P()
            p.position = t.translation
            p.orientation = t.rotation
            return p
        except Exception:
            return None

    def _plan(self, m):
        self.plan_len = len(m.poses)

    def spin(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    gx, gy = float(sys.argv[1]), float(sys.argv[2])
    gyaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    timeout = float(sys.argv[4]) if len(sys.argv) > 4 else 90.0

    rclpy.init()
    n = NavGoal()
    n.spin(2.0)

    if not n.cli.wait_for_server(timeout_sec=10.0):
        print('navigate_to_pose action server NOT available')
        return 1

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = n.get_clock().now().to_msg()
    goal.pose.pose.position.x = gx
    goal.pose.pose.position.y = gy
    qx, qy, qz, qw = yaw_to_quat(gyaw)
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = qz, qw

    start = n.pose
    if start:
        print(f'start  x={start.position.x:+.2f} y={start.position.y:+.2f} '
              f'yaw={math.degrees(quat_to_yaw(start.orientation)):+.0f}deg')
    print(f'goal   x={gx:+.2f} y={gy:+.2f} yaw={math.degrees(gyaw):+.0f}deg\n')

    fut = n.cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        print('goal REJECTED')
        return 1
    print('goal accepted\n')
    print(f'{"t":>5} {"dist":>6} {"yaw":>7} {"vyaw":>7} {"plan":>5}')
    print('-' * 36)

    res_fut = handle.get_result_async()
    t0 = time.monotonic()
    last = 0
    while rclpy.ok() and not res_fut.done():
        rclpy.spin_once(n, timeout_sec=0.1)
        t = time.monotonic() - t0
        if t - last >= 1.0:
            last = t
            if n.pose:
                d = math.hypot(gx - n.pose.position.x, gy - n.pose.position.y)
                print(f'{t:5.0f} {d:6.2f} '
                      f'{math.degrees(quat_to_yaw(n.pose.orientation)):+7.0f} '
                      f'{n.vyaw:+7.3f} {n.plan_len:5d}')
        if t > timeout:
            print('\nTIMEOUT — cancelling')
            handle.cancel_goal_async()
            n.spin(3.0)
            break

    if res_fut.done():
        code = res_fut.result().status
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        name = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(code, str(code))
        print(f'\nresult: {name}')

    if n.pose:
        d = math.hypot(gx - n.pose.position.x, gy - n.pose.position.y)
        print(f'final  x={n.pose.position.x:+.2f} y={n.pose.position.y:+.2f} '
              f'yaw={math.degrees(quat_to_yaw(n.pose.orientation)):+.0f}deg  '
              f'dist-to-goal={d:.2f} m')
    print(f'peak |vyaw| during run: {n.max_vyaw:.3f} rad/s')

    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
