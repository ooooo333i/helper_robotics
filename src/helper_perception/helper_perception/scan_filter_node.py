import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFilterNode(Node):
    """Filter LiDAR scan data before downstream perception nodes use it."""

    def __init__(self):
        super().__init__('scan_filter_node')

        self.declare_parameter('angle_min_deg', -90.0)
        self.declare_parameter('angle_max_deg', 90.0)
        self.declare_parameter('min_valid_range', 0.15)
        self.declare_parameter('max_valid_range', 8.0)
        self.declare_parameter(
            'input_scan_topic',
            '/perception/scan/front/raw',
        )
        self.declare_parameter(
            'output_scan_topic',
            '/perception/scan/front',
        )

        input_scan_topic = self.get_parameter('input_scan_topic').value
        output_scan_topic = self.get_parameter('output_scan_topic').value

        self.publisher = self.create_publisher(
            LaserScan,
            output_scan_topic,
            10,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            input_scan_topic,
            self.scan_callback,
            10,
        )

    def scan_callback(self, msg):
        angle_min_rad = math.radians(
            self.get_parameter('angle_min_deg').value
        )
        angle_max_rad = math.radians(
            self.get_parameter('angle_max_deg').value
        )
        min_valid_range = self.get_parameter('min_valid_range').value
        max_valid_range = self.get_parameter('max_valid_range').value

        # Copy the original message so header and scan metadata stay unchanged.
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.intensities = msg.intensities

        filtered_ranges = []
        current_angle = msg.angle_min

        for distance in msg.ranges:
            in_angle_window = angle_min_rad <= current_angle <= angle_max_rad
            valid_distance = (
                math.isfinite(distance)
                and min_valid_range <= distance <= max_valid_range
            )

            # Invalid or out-of-window samples are ignored by publishing +inf.
            if in_angle_window and valid_distance:
                filtered_ranges.append(distance)
            else:
                filtered_ranges.append(math.inf)

            current_angle += msg.angle_increment

        filtered_msg.ranges = filtered_ranges
        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
