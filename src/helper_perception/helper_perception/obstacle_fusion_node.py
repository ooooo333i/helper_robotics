import math

import rclpy
from helper_msgs.msg import ObstacleDecision
from rclpy.node import Node


class ObstacleFusionNode(Node):
    """Fuse LiDAR and depth obstacle decisions into one safety topic."""

    def __init__(self):
        super().__init__('obstacle_fusion_node')

        self.declare_parameter('range_topic', '/perception/obstacle/range')
        self.declare_parameter('depth_topic', '/perception/obstacle/depth')
        self.declare_parameter('output_topic', '/perception/obstacle/fused')
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('prefer_range_distance', True)
        self.declare_parameter('timeout_is_obstacle', True)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.range_msg = None
        self.depth_msg = None
        self.range_time = None
        self.depth_time = None

        self.publisher = self.create_publisher(
            ObstacleDecision,
            self.get_parameter('output_topic').value,
            10,
        )
        self.range_sub = self.create_subscription(
            ObstacleDecision,
            self.get_parameter('range_topic').value,
            self.range_callback,
            10,
        )
        self.depth_sub = self.create_subscription(
            ObstacleDecision,
            self.get_parameter('depth_topic').value,
            self.depth_callback,
            10,
        )

        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish)

    def range_callback(self, msg):
        self.range_msg = msg
        self.range_time = self.get_clock().now()

    def depth_callback(self, msg):
        self.depth_msg = msg
        self.depth_time = self.get_clock().now()

    def publish(self):
        now = self.get_clock().now()
        range_fresh = self.is_fresh(self.range_time, now)
        depth_fresh = self.is_fresh(self.depth_time, now)
        timeout_is_obstacle = self.get_parameter('timeout_is_obstacle').value
        fresh_inputs = []

        if range_fresh and self.range_msg is not None:
            fresh_inputs.append(('range', self.range_msg))
        if depth_fresh and self.depth_msg is not None:
            fresh_inputs.append(('depth', self.depth_msg))

        if not fresh_inputs:
            decision = 'obstacle' if timeout_is_obstacle else 'unknown'
            contributors = []
        elif any(msg.decision == 'obstacle' for _, msg in fresh_inputs):
            decision = 'obstacle'
            contributors = [
                item for item in fresh_inputs
                if item[1].decision == decision
            ]
        elif any(msg.decision == 'unknown' for _, msg in fresh_inputs):
            decision = 'unknown'
            contributors = [
                item for item in fresh_inputs
                if item[1].decision == decision
            ]
        else:
            decision = 'clear'
            contributors = fresh_inputs

        distance = self.pick_distance(contributors)
        heights = [
            float(msg.height)
            for _, msg in contributors
            if math.isfinite(msg.height)
        ]
        height = max(heights, default=0.0)
        is_dynamic = any(msg.is_dynamic for _, msg in contributors)

        msg = ObstacleDecision()
        msg.obstacle_type = 'fused'
        msg.decision = decision
        msg.distance = float(distance)
        msg.height = height
        msg.is_dynamic = is_dynamic
        self.publisher.publish(msg)

    def is_fresh(self, stamp, now):
        if stamp is None:
            return False
        timeout = self.get_parameter('input_timeout_sec').value
        return (now - stamp).nanoseconds <= int(timeout * 1_000_000_000)

    def pick_distance(self, contributors):
        prefer_range = self.get_parameter('prefer_range_distance').value
        range_distance = math.inf
        distances = []
        for source, msg in contributors:
            distance = float(msg.distance)
            if not math.isfinite(distance):
                continue
            distances.append(distance)
            if source == 'range':
                range_distance = distance

        if prefer_range and math.isfinite(range_distance):
            return range_distance
        if distances:
            return min(distances)
        return math.inf


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
