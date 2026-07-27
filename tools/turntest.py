#!/usr/bin/env python3
"""Measure achieved yaw rate against commanded, through the real Nav2 chain.

Publishes to /cmd_vel_nav so the command passes through velocity_smoother
(exercising the new accel limits) on its way to /cmd_vel and the firmware —
the same path Nav2 itself uses.

Ground truth is the IMU gyro, which is independent of both the motors and the
laser odometry. /odometry/filtered is reported alongside it so you can see
whether the EKF fusion agrees with the gyro.

The point is not just "does it reach 0.5 rad/s" but "is the response
PROPORTIONAL" — the old tuning saturated everything above 0.2 rad/s, so every
turn came out identical regardless of what was asked for.
"""
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu

SETTLE_S = 1.2      # let the smoother ramp up before measuring
MEASURE_S = 2.0
PAUSE_S = 1.5


class TurnTest(Node):
    def __init__(self):
        super().__init__('turn_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=50)
        self.create_subscription(Imu, '/imu/data_raw', self._imu_cb, sensor_qos)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._odom_cb, 20)
        self.gz = []
        self.oz = []
        self.collect = False

    def _imu_cb(self, m):
        if self.collect:
            self.gz.append(m.angular_velocity.z)

    def _odom_cb(self, m):
        if self.collect:
            self.oz.append(m.twist.twist.angular.z)

    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def _send(self, ang):
        t = Twist()
        t.angular.z = float(ang)
        self.pub.publish(t)

    def stop(self):
        for _ in range(10):
            self._send(0.0)
            self._spin_for(0.05)

    def run_one(self, cmd):
        # hold the command continuously; the smoother needs a live stream
        self.gz, self.oz = [], []
        end_settle = time.monotonic() + SETTLE_S
        while time.monotonic() < end_settle and rclpy.ok():
            self._send(cmd)
            self._spin_for(0.05)

        self.collect = True
        end_meas = time.monotonic() + MEASURE_S
        while time.monotonic() < end_meas and rclpy.ok():
            self._send(cmd)
            self._spin_for(0.05)
        self.collect = False

        self.stop()
        self._spin_for(PAUSE_S)

        gz = statistics.mean(self.gz) if self.gz else float('nan')
        gz_sd = statistics.pstdev(self.gz) if len(self.gz) > 1 else float('nan')
        oz = statistics.mean(self.oz) if self.oz else float('nan')
        return gz, gz_sd, oz, len(self.gz)


def main():
    cmds = [float(x) for x in sys.argv[1:]] or [0.15, 0.25, 0.35, 0.45, 0.50]
    rclpy.init()
    node = TurnTest()
    # let subscriptions connect
    node._spin_for(2.0)

    print(f'{"cmd":>6} {"gyro":>8} {"ratio":>7} {"sd":>7} {"ekf":>8} {"n":>5}'
          f'   (rad/s)')
    print('-' * 52)
    results = []
    try:
        for c in cmds:
            gz, sd, oz, n = node.run_one(c)
            ratio = gz / c if c else float('nan')
            results.append((c, gz, ratio))
            print(f'{c:6.2f} {gz:8.3f} {ratio:7.2f} {sd:7.3f} {oz:8.3f} {n:5d}')
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()

    print('\nProportionality check (achieved should rise with commanded):')
    ok = all(results[i][1] < results[i + 1][1] for i in range(len(results) - 1))
    print(f'  monotonic: {"YES" if ok else "NO — response is saturating"}')
    if len(results) >= 2:
        lo, hi = results[0], results[-1]
        print(f'  {lo[0]:.2f} -> {lo[1]:.3f} rad/s, '
              f'{hi[0]:.2f} -> {hi[1]:.3f} rad/s '
              f'(span x{hi[1]/lo[1]:.1f} for a x{hi[0]/lo[0]:.1f} command span)')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
