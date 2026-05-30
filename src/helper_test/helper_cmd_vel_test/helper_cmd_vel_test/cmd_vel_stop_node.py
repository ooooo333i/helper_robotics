import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelStop(Node):
    def __init__(self):
        super().__init__('cmd_vel_stop')

        self.declare_parameter('topic', '/control/cmd_vel_safe')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('stop_publish_count', 20)

        self.topic = self.get_parameter('topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.stop_publish_count = int(
            self.get_parameter('stop_publish_count').value
        )
        self.stop_count = 0

        self.pub = self.create_publisher(Twist, self.topic, 10)
        timer_period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            'publishing stop Twist to %s count=%d'
            % (self.topic, self.stop_publish_count)
        )

    def timer_callback(self):
        self.pub.publish(Twist())
        self.stop_count += 1

        if self.stop_count >= self.stop_publish_count:
            self.get_logger().info('stop command published')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelStop()

    try:
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
