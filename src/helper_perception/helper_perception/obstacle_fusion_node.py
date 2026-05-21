import math

import rclpy
from helper_msgs.msg import ObstacleDecision
from rclpy.node import Node


class ObstacleFusionNode(Node):
    """Fuse LiDAR and depth obstacle decisions into one safety topic."""

    def __init__(self):
        super().__init__('obstacle_fusion_node')

        self.declare_parameter('lidar_topic', '/perception/obstacle/lidar')
        self.declare_parameter('depth_topic', '/perception/obstacle/depth')
        self.declare_parameter('output_topic', '/perception/obstacle/fused')
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('prefer_lidar_distance', True)
        self.declare_parameter('timeout_is_obstacle', True)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.lidar_msg = None
        self.depth_msg = None
        self.lidar_time = None
        self.depth_time = None

        self.publisher = self.create_publisher(
            ObstacleDecision,
            self.get_parameter('output_topic').value,
            10,
        )
        self.lidar_sub = self.create_subscription(
            ObstacleDecision,
            self.get_parameter('lidar_topic').value,
            self.lidar_callback,
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

    def lidar_callback(self, msg):
        self.lidar_msg = msg
        self.lidar_time = self.get_clock().now()

    def depth_callback(self, msg):
        self.depth_msg = msg
        self.depth_time = self.get_clock().now()

    def publish(self):
        now = self.get_clock().now()
        lidar_fresh = self.is_fresh(self.lidar_time, now)
        depth_fresh = self.is_fresh(self.depth_time, now)
        timeout_is_obstacle = self.get_parameter('timeout_is_obstacle').value

        if not lidar_fresh or not depth_fresh:
            decision = 'obstacle' if timeout_is_obstacle else 'unknown'
            distance = self.pick_distance(lidar_fresh, depth_fresh)
        elif (
            self.lidar_msg.decision == 'obstacle'
            or self.depth_msg.decision == 'obstacle'
        ):
            decision = 'obstacle'
            distance = self.pick_distance(lidar_fresh, depth_fresh)
        elif (
            self.lidar_msg.decision == 'unknown'
            or self.depth_msg.decision == 'unknown'
        ):
            decision = 'unknown'
            distance = self.pick_distance(lidar_fresh, depth_fresh)
        else:
            decision = 'clear'
            distance = self.pick_distance(lidar_fresh, depth_fresh)

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

    def pick_distance(self, lidar_fresh, depth_fresh):
        prefer_lidar = self.get_parameter('prefer_lidar_distance').value
        lidar_distance = (
            self.lidar_msg.distance
            if lidar_fresh and self.lidar_msg is not None
            else math.inf
        )
        depth_distance = (
            self.depth_msg.distance
            if depth_fresh and self.depth_msg is not None
            else math.inf
        )

        if prefer_lidar and math.isfinite(lidar_distance):
            return lidar_distance
        if math.isfinite(lidar_distance) or math.isfinite(depth_distance):
            return min(lidar_distance, depth_distance)
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
