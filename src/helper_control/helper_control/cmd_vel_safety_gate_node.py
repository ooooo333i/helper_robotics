import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class CmdVelSafetyGateNode(Node):
    """Pass cmd_vel through unless behavior requests a hard stop."""

    RELEASE_BEHAVIORS = {'run', 'overcome', 'avoid'}

    def __init__(self):
        super().__init__('cmd_vel_safety_gate_node')

        self.declare_parameter('input_cmd_vel_topic', '/control/cmd_vel_smoothed')
        self.declare_parameter('output_cmd_vel_topic', '/control/cmd_vel_safe')
        self.declare_parameter('behavior_topic', '/planning/behavior_state')
        self.declare_parameter('stop_publish_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)

        self.hard_stopped = False
        self.last_cmd_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter('output_cmd_vel_topic').value,
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter('input_cmd_vel_topic').value,
            self.cmd_vel_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('behavior_topic').value,
            self.behavior_callback,
            10,
        )

        rate = float(self.get_parameter('stop_publish_rate_hz').value)
        self.create_timer(1.0 / max(rate, 1.0), self.timer_callback)

        self.get_logger().info(
            f'gating {self.get_parameter("input_cmd_vel_topic").value} '
            f'to {self.get_parameter("output_cmd_vel_topic").value}'
        )

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        if self.hard_stopped:
            self.publish_stop()
            return

        self.cmd_pub.publish(msg)

    def behavior_callback(self, msg):
        behavior = msg.data.strip().lower()
        if behavior == 'stop':
            if not self.hard_stopped:
                self.get_logger().warn('hard stop active')
            self.hard_stopped = True
            self.publish_stop()
        elif behavior in self.RELEASE_BEHAVIORS:
            if self.hard_stopped:
                self.get_logger().info(f'hard stop released by {behavior}')
            self.hard_stopped = False

    def timer_callback(self):
        if self.hard_stopped:
            self.publish_stop()
            return

        elapsed = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds / 1e9
        timeout = float(self.get_parameter('cmd_timeout_sec').value)
        if elapsed > timeout:
            self.publish_stop()

    def publish_stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafetyGateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
