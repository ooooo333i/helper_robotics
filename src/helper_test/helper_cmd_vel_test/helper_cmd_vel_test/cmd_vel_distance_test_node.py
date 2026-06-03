import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelDistanceTest(Node):
    def __init__(self):
        super().__init__('cmd_vel_distance_test')

        self.declare_parameter('topic', '/control/cmd_vel_safe')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('distance', 0.5)
        self.declare_parameter('linear_x', 0.1)
        self.declare_parameter('angular_z', 0.0)
        self.declare_parameter('stop_publish_count', 10)

        self.topic = self.get_parameter('topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.distance = float(self.get_parameter('distance').value)
        self.linear_x = float(self.get_parameter('linear_x').value)
        self.angular_z = float(self.get_parameter('angular_z').value)
        self.stop_publish_count = int(
            self.get_parameter('stop_publish_count').value
        )

        self.pub = self.create_publisher(Twist, self.topic, 10)
        self.start_time = self.get_clock().now()
        self.stop_count = 0
        self.is_stopping = False
        self.done = False

        if self.linear_x <= 0.0:
            self.get_logger().error('linear_x must be greater than 0.0')
            rclpy.shutdown()
            return

        self.drive_duration = self.distance / self.linear_x
        timer_period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            'publishing %.3f m/s for %.3f m to %s, duration %.3f sec'
            % (
                self.linear_x,
                self.distance,
                self.topic,
                self.drive_duration,
            )
        )

    def timer_callback(self):
        if self.is_stopping:
            self.publish_stop()
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= self.drive_duration:
            self.is_stopping = True
            self.publish_stop()
            return

        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.pub.publish(msg)

    def publish_stop(self):
        self.pub.publish(Twist())
        self.stop_count += 1

        if self.stop_count >= self.stop_publish_count:
            self.get_logger().info('distance test done, published stop command')
            self.done = True
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDistanceTest()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.publish_stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
