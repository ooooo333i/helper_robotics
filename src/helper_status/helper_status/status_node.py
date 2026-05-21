import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from helper_msgs.msg import RobotStatus


class StatusNode(Node):
    def __init__(self):
        super().__init__('status_node')

        self.sub_odom = self.create_subscription(
            Odometry,
            '/control/odom',
            self.odom_callback,
            10
        )

        self.pub_status = self.create_publisher(
            RobotStatus,
            '/control/status',
            10
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.timer = self.create_timer(0.1, self.publish_status)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # theta는 나중에 추가 (quaternion 변환 필요)

    def publish_status(self):
        msg = RobotStatus()

        msg.robot_id = "robot_01"
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        msg.battery_percent = 80.0
        msg.current_action = "idle"
        msg.status = "normal"

        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
    rclpy.spin(node)
    rclpy.shutdown()
