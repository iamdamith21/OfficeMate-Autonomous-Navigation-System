#!/usr/bin/env python3
"""
api_adapter.py — web app telemetry adapter.

Translates native ROS 2 topics into the browser contract expected by the web dashboard:
  /battery/state     -> /battery_level       (std_msgs/Float32)
  /navigate_to_pose/status -> /nav/status     (std_msgs/String)
  /ultrasonic/range  -> /ultrasonic/distance  (std_msgs/Float32)
  /doors/state       -> /locker/status        (std_msgs/Bool)
"""
import rclpy
from action_msgs.msg import GoalStatusArray
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class ApiAdapter(Node):
    def __init__(self):
        super().__init__('api_adapter')

        # Publishers
        self.pub_bat = self.create_publisher(Float32, '/battery_level', 10)
        self.pub_nav = self.create_publisher(String, '/nav/status', 10)
        self.pub_ultra = self.create_publisher(Float32, '/ultrasonic/distance', 10)
        self.pub_locker = self.create_publisher(Bool, '/locker/status', 10)

        # Default telemetry values
        self.battery_val = 100.0
        self.nav_val = "Idle"
        self.ultra_val = 999.0
        self.locker_val = False

        # Subscriptions
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_cb, 10)

        # Heartbeat timer (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.get_logger().info("API Adapter started: bridging telemetry to Web App.")

    def nav_status_cb(self, msg):
        if not msg.status_list:
            self.nav_val = "Idle"
            return
        last_status = msg.status_list[-1].status
        # Status code 2: EXECUTING, 4: SUCCEEDED, 5: CANCELED, 6: ABORTED
        if last_status == 2:
            self.nav_val = "Navigating"
        elif last_status == 4:
            self.nav_val = "Goal Reached"
        elif last_status == 5:
            self.nav_val = "Canceled"
        elif last_status == 6:
            self.nav_val = "Goal Failed"

    def timer_cb(self):
        b = Float32()
        b.data = self.battery_val
        self.pub_bat.publish(b)

        n = String()
        n.data = self.nav_val
        self.pub_nav.publish(n)

        u = Float32()
        u.data = self.ultra_val
        self.pub_ultra.publish(u)

        l = Bool()
        l.data = self.locker_val
        self.pub_locker.publish(l)


def main(args=None):
    rclpy.init(args=args)
    node = ApiAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
