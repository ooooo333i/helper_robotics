import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelTurnTest(Node):
    def __init__(self):
        super().__init__('cmd_vel_turn_test')

        self.declare_parameter('topic', '/control/cmd_vel_safe')
        self.declare_parameter('mode', 'spin')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('linear_x', 0.1)
        self.declare_parameter('angular_z', 0.3)
        self.declare_parameter('target_yaw_deg', 90.0)
        self.declare_parameter('stop_publish_count', 10)

        self.topic = self.get_parameter('topic').value
        self.mode = self.get_parameter('mode').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.linear_x = float(self.get_parameter('linear_x').value)
        self.angular_z = float(self.get_parameter('angular_z').value)
        self.target_yaw_deg = float(self.get_parameter('target_yaw_deg').value)
        self.stop_publish_count = int(
            self.get_parameter('stop_publish_count').value
        )

        self.pub = self.create_publisher(Twist, self.topic, 10)
        self.start_time = self.get_clock().now()
        self.stop_count = 0
        self.is_stopping = False

        if self.angular_z == 0.0:
            self.get_logger().error('angular_z must not be 0.0')
            rclpy.shutdown()
            return

        if self.mode not in ('spin', 'arc'):
            self.get_logger().error('mode must be spin or arc')
            rclpy.shutdown()
            return

        target_yaw_rad = math.radians(abs(self.target_yaw_deg))
        self.drive_duration = target_yaw_rad / abs(self.angular_z)
        timer_period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        if self.mode == 'spin':
            self.command_linear_x = 0.0
        else:
            self.command_linear_x = self.linear_x

        self.get_logger().info(
            (
                'publishing mode=%s linear_x=%.3f angular_z=%.3f '
                'for %.1f deg to %s, duration %.3f sec'
            )
            % (
                self.mode,
                self.command_linear_x,
                self.angular_z,
                self.target_yaw_deg,
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
        msg.linear.x = self.command_linear_x
        msg.angular.z = self.angular_z
        self.pub.publish(msg)

    def publish_stop(self):
        self.pub.publish(Twist())
        self.stop_count += 1

        if self.stop_count >= self.stop_publish_count:
            self.get_logger().info('turn test done, published stop command')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTurnTest()

    try:
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
