import math

from helper_msgs.msg import ObstacleDecision
import rclpy

from helper_perception.depth_obstacle_detector_node import (
    DepthObstacleDetectorNode,
)
from helper_perception.obstacle_fusion_node import ObstacleFusionNode


class CapturePublisher:

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_depth_decision_requires_consecutive_frames():
    rclpy.init()
    node = DepthObstacleDetectorNode()
    try:
        for _ in range(2):
            decision, _, _ = node.stabilize_decision(
                'obstacle',
                0.5,
                0.08,
            )
            assert decision == 'unknown'

        decision, distance, height = node.stabilize_decision(
            'obstacle',
            0.5,
            0.08,
        )
        assert decision == 'obstacle'
        assert distance == 0.5
        assert height == 0.08

        decision, _, _ = node.stabilize_decision(
            'clear',
            math.inf,
            0.0,
        )
        assert decision == 'obstacle'

        decision, distance, height = node.stabilize_decision(
            'clear',
            math.inf,
            0.0,
        )
        assert decision == 'clear'
        assert math.isinf(distance)
        assert height == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_fusion_preserves_contributing_depth_fields():
    rclpy.init()
    node = ObstacleFusionNode()
    capture = CapturePublisher()
    node.publisher = capture
    try:
        range_msg = ObstacleDecision()
        range_msg.obstacle_type = 'range'
        range_msg.decision = 'clear'
        range_msg.distance = 1.2

        depth_msg = ObstacleDecision()
        depth_msg.obstacle_type = 'depth'
        depth_msg.decision = 'obstacle'
        depth_msg.distance = 0.4
        depth_msg.height = 0.08
        depth_msg.is_dynamic = True

        now = node.get_clock().now()
        node.range_msg = range_msg
        node.range_time = now
        node.depth_msg = depth_msg
        node.depth_time = now
        node.publish()

        fused = capture.messages[-1]
        assert fused.decision == 'obstacle'
        assert fused.distance == 0.4
        assert fused.height == 0.08
        assert fused.is_dynamic is True
    finally:
        node.destroy_node()
        rclpy.shutdown()
