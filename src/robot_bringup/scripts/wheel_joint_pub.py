#!/usr/bin/env python3
"""
wheel_joint_pub.py — publishes /wheel_joint_states so joint_state_publisher
can merge them into /joint_states.

4WD skid-steer: the two left wheels (front_left, rear_left) share one angle,
the two right wheels (front_right, rear_right) share the other. Angles are
integrated from the EKF-fused odometry (/odometry/filtered). The laser joint
stays at 0.0 (the laser frame must be fixed).

URDF constants (from properties.xacro):
  wheel_radius   = 0.065 m
  wheel_y_offset = 0.165 m  (centre of wheel from robot centreline)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

WHEEL_RADIUS   = 0.065   # m
WHEEL_Y_OFFSET = 0.165   # m

class WheelJointPub(Node):
    def __init__(self):
        super().__init__('wheel_joint_publisher')
        self.left_angle  = 0.0
        self.right_angle = 0.0
        self._last_ns    = None

        self.sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb,
            rclpy.qos.QoSProfile(depth=5))
        self.pub = self.create_publisher(
            JointState, '/wheel_joint_states', 10)
        # Publish at 20 Hz — fast enough to keep joint_state_publisher's
        # merged output smooth.
        self.create_timer(0.05, self._publish)

    def _odom_cb(self, msg: Odometry):
        now_ns = self.get_clock().now().nanoseconds
        if self._last_ns is None:
            self._last_ns = now_ns
            return
        dt = (now_ns - self._last_ns) * 1e-9
        self._last_ns = now_ns
        if dt <= 0.0 or dt > 1.0:
            return
        v  = msg.twist.twist.linear.x
        w  = msg.twist.twist.angular.z
        self.left_angle  += ((v - w * WHEEL_Y_OFFSET) / WHEEL_RADIUS) * dt
        self.right_angle += ((v + w * WHEEL_Y_OFFSET) / WHEEL_RADIUS) * dt

    def _publish(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ['front_left_wheel_joint', 'rear_left_wheel_joint',
                       'front_right_wheel_joint', 'rear_right_wheel_joint']
        js.position = [self.left_angle, self.left_angle,
                       self.right_angle, self.right_angle]
        self.pub.publish(js)

def main():
    rclpy.init()
    try:
        rclpy.spin(WheelJointPub())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
