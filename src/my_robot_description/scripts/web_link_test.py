#!/usr/bin/env python3
"""
web_link_test.py — ROS ↔ web-backend round-trip diagnostic.

Proves the plumbing the Phase-4 robot agent will use, without needing
robot hardware:

  ROS → web : subscribes /battery/state (sensor_msgs/BatteryState) and
              POSTs batteryLevel to <backend>/api/robot/update
  web → ROS : polls <backend>/api/robot/status every 2 s and republishes
              the JSON on /web/robot_status (std_msgs/String)

Run on the Pi (backend running on the laptop):
  python3 web_link_test.py http://<laptop-ip>:5000

Then, in other terminals:
  ros2 topic pub /battery/state sensor_msgs/msg/BatteryState \
      "{voltage: 11.4, percentage: 0.67}" -r 1        # fake battery → web
  ros2 topic echo /web/robot_status                   # web state → ROS

Uses only the Python stdlib for HTTP (urllib) — no extra deps on the Pi.
"""
import json
import sys
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

POLL_PERIOD_S = 2.0
HTTP_TIMEOUT_S = 4.0


class WebLinkTest(Node):
    def __init__(self, base_url: str):
        super().__init__('web_link_test')
        self.base_url = base_url.rstrip('/')
        self.last_sent_pct = None

        self.create_subscription(BatteryState, '/battery/state',
                                 self._battery_cb, 5)
        self.pub_status = self.create_publisher(String, '/web/robot_status', 5)
        self.create_timer(POLL_PERIOD_S, self._poll_status)
        self.get_logger().info(f'Testing against {self.base_url}')

    # ── ROS → web ────────────────────────────────────────────────────────────
    def _battery_cb(self, msg: BatteryState):
        pct = round(msg.percentage * 100.0, 1)
        if pct == self.last_sent_pct:
            return
        body = json.dumps({'batteryLevel': pct}).encode()
        req = urllib.request.Request(
            self.base_url + '/api/robot/update', data=body,
            headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as r:
                ok = json.load(r).get('success', False)
            self.last_sent_pct = pct
            self.get_logger().info(
                f'ROS→web: battery {pct}% POSTed (success={ok})')
        except (urllib.error.URLError, OSError, ValueError) as e:
            self.get_logger().error(f'ROS→web POST failed: {e}')

    # ── web → ROS ────────────────────────────────────────────────────────────
    def _poll_status(self):
        try:
            with urllib.request.urlopen(
                    self.base_url + '/api/robot/status',
                    timeout=HTTP_TIMEOUT_S) as r:
                data = json.load(r).get('data', {})
        except (urllib.error.URLError, OSError, ValueError) as e:
            self.get_logger().error(f'web→ROS GET failed: {e}')
            return
        brief = {k: data.get(k) for k in
                 ('status', 'currentLocation', 'batteryLevel')}
        self.pub_status.publish(String(data=json.dumps(brief)))


def main():
    if len(sys.argv) < 2:
        sys.exit('usage: web_link_test.py http://<backend-host>:5000')
    rclpy.init()
    try:
        rclpy.spin(WebLinkTest(sys.argv[1]))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
