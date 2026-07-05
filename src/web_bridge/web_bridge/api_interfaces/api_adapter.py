#!/usr/bin/env python3
"""api_adapter — placeholder (fleshed out in Phase 6).

Bridges the OfficeMate web app (via rosbridge) to the mission action/topics.
"""
import rclpy
from rclpy.node import Node


class ApiAdapter(Node):
    def __init__(self):
        super().__init__('api_adapter')
        self.get_logger().info('api_adapter placeholder up (Phase 6 TODO)')


def main():
    rclpy.init()
    try:
        rclpy.spin(ApiAdapter())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
