import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')

        self.pub = self.create_publisher(Odometry, '/control/odom', 10)
        self.timer = self.create_timer(0.5, self.publish_odom)

        self.x = 0.0

    def publish_odom(self):
        msg = Odometry()

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = 0.0

        self.x += 0.1

        self.pub.publish(msg)
        self.get_logger().info(f"Publishing x={self.x}")


def main():
    rclpy.init()
    node = FakeOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
