import math

import rclpy
from helper_msgs.msg import ObstacleDecision
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleDetectorNode(Node):
    """Detect whether an obstacle exists in the configured LiDAR region."""

    def __init__(self):
        super().__init__('obstacle_detector_node')

        self.declare_parameter('detection_angle_min_deg', -30.0)
        self.declare_parameter('detection_angle_max_deg', 30.0)
        self.declare_parameter('obstacle_distance_threshold', 1.0)
        self.declare_parameter(
            'input_scan_topic',
            '/perception/scan/filtered',
        )
        self.declare_parameter(
            'output_obstacle_topic',
            '/perception/obstacle/range',
        )

        input_scan_topic = self.get_parameter('input_scan_topic').value
        output_obstacle_topic = self.get_parameter(
            'output_obstacle_topic'
        ).value

        self.publisher = self.create_publisher(
            ObstacleDecision,
            output_obstacle_topic,
            10,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            input_scan_topic,
            self.scan_callback,
            10,
        )

        # ROS time is used so simulation time also works correctly.
        self.last_log_time = self.get_clock().now()

    def scan_callback(self, msg):
        angle_min_rad = math.radians(
            self.get_parameter('detection_angle_min_deg').value
        )
        angle_max_rad = math.radians(
            self.get_parameter('detection_angle_max_deg').value
        )
        threshold = self.get_parameter('obstacle_distance_threshold').value

        valid_distances = []
        current_angle = msg.angle_min

        for distance in msg.ranges:
            in_detection_window = self.angle_in_window(
                current_angle,
                angle_min_rad,
                angle_max_rad,
            )

            # Filtered scans use +inf for ignored samples, so keep finite values only.
            if in_detection_window and math.isfinite(distance):
                valid_distances.append(distance)

            current_angle += msg.angle_increment

        min_distance = min(valid_distances) if valid_distances else math.inf
        obstacle_detected = min_distance <= threshold

        result_msg = ObstacleDecision()
        result_msg.obstacle_type = 'range'
        result_msg.decision = 'obstacle' if obstacle_detected else 'clear'
        result_msg.distance = float(min_distance)
        result_msg.height = 0.0
        result_msg.is_dynamic = False
        self.publisher.publish(result_msg)

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds >= 1_000_000_000:
            min_distance_text = (
                f'{min_distance:.3f} m'
                if math.isfinite(min_distance)
                else 'inf'
            )
            self.get_logger().info(
                f'min_distance={min_distance_text}, '
                f'obstacle_detected={obstacle_detected}'
            )
            self.last_log_time = now

    @staticmethod
    def angle_in_window(angle, angle_min, angle_max):
        if angle_min <= angle_max:
            return angle_min <= angle <= angle_max

        return angle >= angle_min or angle <= angle_max


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
