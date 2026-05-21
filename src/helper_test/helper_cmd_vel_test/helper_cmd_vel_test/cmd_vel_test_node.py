import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelTest(Node):
    def __init__(self):
        super().__init__('cmd_vel_test')

        self.declare_parameter('topic', '/control/cmd_vel')
        self.declare_parameter('mode', 'constant')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('linear_x', 0.0)
        self.declare_parameter('angular_z', 0.0)
        self.declare_parameter('step_duration', 2.0)

        self.topic = self.get_parameter('topic').value
        self.mode = self.get_parameter('mode').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.linear_x = self.get_parameter('linear_x').value
        self.angular_z = self.get_parameter('angular_z').value
        self.step_duration = self.get_parameter('step_duration').value

        self.pub = self.create_publisher(Twist, self.topic, 10)
        self.start_time = self.get_clock().now()

        timer_period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            'publishing Twist to %s mode=%s linear_x=%.3f angular_z=%.3f'
            % (self.topic, self.mode, self.linear_x, self.angular_z)
        )

    def timer_callback(self):
        if self.mode == 'sequence':
            msg = self.build_sequence_msg()
        else:
            msg = self.build_constant_msg()

        self.pub.publish(msg)

    def build_constant_msg(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        return msg

    def build_sequence_msg(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        step = int(elapsed / max(self.step_duration, 0.1)) % 6

        msg = Twist()
        if step == 0:
            msg.linear.x = self.linear_x
        elif step == 2:
            msg.angular.z = self.angular_z
        elif step == 4:
            msg.angular.z = -self.angular_z

        return msg

    def publish_stop(self):
        stop_msg = Twist()
        for _ in range(3):
            self.pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
