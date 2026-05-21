import math

import numpy as np
import rclpy
from helper_msgs.msg import ObstacleDecision
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthObstacleDetectorNode(Node):
    """Detect nearby obstacles from a configurable depth-image ROI."""

    def __init__(self):
        super().__init__('depth_obstacle_detector_node')

        self.declare_parameter(
            'input_depth_topic',
            '/perception/depth/image_raw',
        )
        self.declare_parameter(
            'output_obstacle_topic',
            '/perception/obstacle/depth',
        )
        self.declare_parameter('roi_x_min_ratio', 0.35)
        self.declare_parameter('roi_x_max_ratio', 0.65)
        self.declare_parameter('roi_y_min_ratio', 0.35)
        self.declare_parameter('roi_y_max_ratio', 0.75)
        self.declare_parameter('obstacle_distance_threshold', 0.8)
        self.declare_parameter('min_valid_depth', 0.2)
        self.declare_parameter('max_valid_depth', 3.0)
        self.declare_parameter('depth_unit_scale', 0.001)
        self.declare_parameter('distance_percentile', 10.0)

        input_depth_topic = self.get_parameter('input_depth_topic').value
        output_obstacle_topic = self.get_parameter(
            'output_obstacle_topic'
        ).value

        self.publisher = self.create_publisher(
            ObstacleDecision,
            output_obstacle_topic,
            10,
        )
        self.subscription = self.create_subscription(
            Image,
            input_depth_topic,
            self.depth_callback,
            10,
        )
        self.last_log_time = self.get_clock().now()

    def depth_callback(self, msg):
        depth = self.image_to_depth_meters(msg)
        if depth is None:
            self.publish_decision('unknown', math.inf)
            return

        roi = self.crop_roi(depth)
        min_valid_depth = self.get_parameter('min_valid_depth').value
        max_valid_depth = self.get_parameter('max_valid_depth').value
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid >= min_valid_depth) & (valid <= max_valid_depth)]

        if valid.size == 0:
            distance = math.inf
            decision = 'unknown'
        else:
            percentile = self.get_parameter('distance_percentile').value
            percentile = min(100.0, max(0.0, percentile))
            distance = float(np.percentile(valid, percentile))
            threshold = self.get_parameter('obstacle_distance_threshold').value
            decision = 'obstacle' if distance <= threshold else 'clear'

        self.publish_decision(decision, distance)
        self.log_status(decision, distance)

    def image_to_depth_meters(self, msg):
        encoding = msg.encoding.upper()
        scale = self.get_parameter('depth_unit_scale').value

        if encoding == '16UC1':
            dtype = np.uint16
        elif encoding == '32FC1':
            dtype = np.float32
        else:
            self.get_logger().warn(
                f'Unsupported depth encoding: {msg.encoding}',
                throttle_duration_sec=2.0,
            )
            return None

        expected_step = msg.width * np.dtype(dtype).itemsize
        if msg.step < expected_step:
            self.get_logger().warn('Depth image step is smaller than expected')
            return None

        raw = np.frombuffer(msg.data, dtype=dtype)
        rows = raw.reshape((msg.height, msg.step // np.dtype(dtype).itemsize))
        depth = rows[:, :msg.width].astype(np.float32)
        return depth * float(scale)

    def crop_roi(self, depth):
        height, width = depth.shape
        x_min_ratio = self.get_parameter('roi_x_min_ratio').value
        x_max_ratio = self.get_parameter('roi_x_max_ratio').value
        y_min_ratio = self.get_parameter('roi_y_min_ratio').value
        y_max_ratio = self.get_parameter('roi_y_max_ratio').value

        x_min = int(width * min(max(x_min_ratio, 0.0), 1.0))
        x_max = int(width * min(max(x_max_ratio, 0.0), 1.0))
        y_min = int(height * min(max(y_min_ratio, 0.0), 1.0))
        y_max = int(height * min(max(y_max_ratio, 0.0), 1.0))

        if x_max <= x_min:
            x_min, x_max = 0, width
        if y_max <= y_min:
            y_min, y_max = 0, height

        return depth[y_min:y_max, x_min:x_max]

    def publish_decision(self, decision, distance):
        msg = ObstacleDecision()
        msg.obstacle_type = 'depth'
        msg.decision = decision
        msg.distance = float(distance)
        msg.height = 0.0
        msg.is_dynamic = False
        self.publisher.publish(msg)

    def log_status(self, decision, distance):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < 1_000_000_000:
            return

        distance_text = (
            f'{distance:.3f} m' if math.isfinite(distance) else 'inf'
        )
        self.get_logger().info(
            f'depth_roi_distance={distance_text}, decision={decision}'
        )
        self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
