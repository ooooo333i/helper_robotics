import math

import rclpy
from helper_msgs.msg import ObstacleDecision
from rclpy.node import Node


class ObstacleDecisionTest(Node):
    """Publish clear/obstacle decisions for motor safety testing."""

    def __init__(self):
        super().__init__('obstacle_decision_test')

        self.declare_parameter('topic', '/perception/obstacle/fused')
        self.declare_parameter('mode', 'sequence')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('clear_duration', 4.0)
        self.declare_parameter('obstacle_duration', 3.0)
        self.declare_parameter('clear_distance', 2.0)
        self.declare_parameter('obstacle_distance', 0.3)

        self.topic = self.get_parameter('topic').value
        publish_rate = self.get_parameter('publish_rate').value
        self.publisher = self.create_publisher(
            ObstacleDecision,
            self.topic,
            10,
        )
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(
            1.0 / max(float(publish_rate), 0.1),
            self.timer_callback,
        )

        self.get_logger().info(f'publishing ObstacleDecision to {self.topic}')

    def timer_callback(self):
        mode = self.get_parameter('mode').value
        if mode == 'obstacle':
            decision = 'obstacle'
        elif mode == 'unknown':
            decision = 'unknown'
        elif mode == 'clear':
            decision = 'clear'
        else:
            decision = self.sequence_decision()

        msg = ObstacleDecision()
        msg.obstacle_type = 'test'
        msg.decision = decision
        msg.distance = self.distance_for_decision(decision)
        msg.height = 0.0
        msg.is_dynamic = False
        self.publisher.publish(msg)

    def sequence_decision(self):
        clear_duration = max(
            float(self.get_parameter('clear_duration').value),
            0.1,
        )
        obstacle_duration = max(
            float(self.get_parameter('obstacle_duration').value),
            0.1,
        )
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        cycle_time = clear_duration + obstacle_duration
        if math.fmod(elapsed, cycle_time) < clear_duration:
            return 'clear'
        return 'obstacle'

    def distance_for_decision(self, decision):
        if decision == 'obstacle':
            return float(self.get_parameter('obstacle_distance').value)
        if decision == 'clear':
            return float(self.get_parameter('clear_distance').value)
        return math.inf


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDecisionTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
