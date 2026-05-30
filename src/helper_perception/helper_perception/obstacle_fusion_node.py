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
        fresh_msgs = []

        if range_fresh and self.range_msg is not None:
            fresh_msgs.append(self.range_msg)
        if depth_fresh and self.depth_msg is not None:
            fresh_msgs.append(self.depth_msg)

        if not fresh_msgs:
            decision = 'obstacle' if timeout_is_obstacle else 'unknown'
            distance = self.pick_distance(range_fresh, depth_fresh)
        elif any(msg.decision == 'obstacle' for msg in fresh_msgs):
            decision = 'obstacle'
            distance = self.pick_distance(range_fresh, depth_fresh)
        elif any(msg.decision == 'unknown' for msg in fresh_msgs):
            decision = 'unknown'
            distance = self.pick_distance(range_fresh, depth_fresh)
        else:
            decision = 'clear'
            distance = self.pick_distance(range_fresh, depth_fresh)

        msg = ObstacleDecision()
        msg.obstacle_type = 'fused'
        msg.decision = decision
        msg.distance = float(distance)
        msg.height = 0.0
        msg.is_dynamic = False
        self.publisher.publish(msg)

    def is_fresh(self, stamp, now):
        if stamp is None:
            return False
        timeout = self.get_parameter('input_timeout_sec').value
        return (now - stamp).nanoseconds <= int(timeout * 1_000_000_000)

    def pick_distance(self, range_fresh, depth_fresh):
        prefer_range = self.get_parameter('prefer_range_distance').value
        range_distance = (
            self.range_msg.distance
            if range_fresh and self.range_msg is not None
            else math.inf
        )
        depth_distance = (
            self.depth_msg.distance
            if depth_fresh and self.depth_msg is not None
            else math.inf
        )

        if prefer_range and math.isfinite(range_distance):
            return range_distance
        if math.isfinite(range_distance) or math.isfinite(depth_distance):
            return min(range_distance, depth_distance)
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
