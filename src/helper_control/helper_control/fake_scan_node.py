import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeScan(Node):
    def __init__(self):
        super().__init__('fake_scan')

        self.scan_pub = self.create_publisher(
            LaserScan,
            '/perception/scan/filtered',
            10,
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        scan = LaserScan()

        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_front'

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)

        scan.time_increment = 0.0
        scan.scan_time = 0.1

        scan.range_min = 0.12
        scan.range_max = 3.5

        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [3.0] * num_readings
        scan.intensities = []

        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    node = FakeScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
