#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math, time

class ScanEmulator(Node):
    def __init__(self):
        super().__init__('scan_emulator')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan) # 10 Hz
        self.get_logger().info('Scan Emulator Node Started (10 Hz)')

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = -2.356194490192345
        scan.angle_max = 2.356194490192345
        scan.angle_increment = 0.01227184630308513
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 12.0
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [3.5] * num_readings
        self.pub.publish(scan)

def main():
    rclpy.init()
    node = ScanEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
