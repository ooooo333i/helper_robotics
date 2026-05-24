import math

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from helper_msgs.msg import ObstacleDecision
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from helper_control.kinematics_engine import KinematicsEngine
from helper_control.md200t_driver import MD200TDriver
from helper_control.robot_parameters import RobotParameters


class MotorDriverNode(Node):
    """ROS 2 bridge from Twist commands to the MD200T motor driver."""

    def __init__(self):
        super().__init__('motor_driver_node')

        self.cfg = RobotParameters()
        self._declare_parameters()
        self._load_parameters()

        self.kinematics = KinematicsEngine(self.cfg)

        self.driver = None
        self.connected = False
        if self.dry_run:
            self.get_logger().warn(
                'dry_run=true: MD200T serial output is disabled.'
            )
        else:
            self.driver = MD200TDriver(
                port=self.cfg.SERIAL_PORT,
                baudrate=self.cfg.BAUD_RATE,
                robot_id=self.cfg.DRIVER_ID,
                max_rpm=self.cfg.MAX_RPM,
            )
            self.connected = self.driver.connect()
            if self.connected:
                self.connected = self.driver.initialize_motor()

        if self.connected:
            self.get_logger().info(
                'MD200T connected on '
                f'{self.cfg.SERIAL_PORT} at {self.cfg.BAUD_RATE} bps'
            )
        elif not self.dry_run:
            self.get_logger().error(
                'MD200T is not connected. RPM commands will not be sent.'
            )

        self.target_left_rpm = 0
        self.target_right_rpm = 0
        self.last_cmd_time = self.get_clock().now()
        self.safety_stop_active = False
        self.obstacle_decision = 'unknown'
        self.obstacle_distance = math.inf
        self.last_obstacle_time = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )
        self.obstacle_sub = self.create_subscription(
            ObstacleDecision,
            self.obstacle_topic,
            self.obstacle_callback,
            10,
        )
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate, 1.0),
            self.control_loop,
        )
        self.odom_timer = self.create_timer(
            1.0 / max(self.cfg.ODOM_PUBLISH_RATE, 1.0),
            self.publish_open_loop_odom,
        )

        self.get_logger().info(f'subscribing to {self.cmd_vel_topic}')
        self.get_logger().info(f'safety obstacle topic: {self.obstacle_topic}')

    def _declare_parameters(self):
        self.declare_parameter('cmd_vel_topic', '/control/cmd_vel_smoothed')
        self.declare_parameter('odom_topic', '/control/odom')
        self.declare_parameter('obstacle_topic', '/perception/obstacle/fused')
        self.declare_parameter('serial_port', self.cfg.SERIAL_PORT)
        self.declare_parameter('baud_rate', self.cfg.BAUD_RATE)
        self.declare_parameter('driver_id', self.cfg.DRIVER_ID)
        self.declare_parameter('wheel_radius', self.cfg.WHEEL_RADIUS)
        self.declare_parameter('track_width', self.cfg.TRACK_WIDTH)
        self.declare_parameter('gear_ratio', self.cfg.GEAR_RATIO)
        self.declare_parameter('max_rpm', self.cfg.MAX_RPM)
        self.declare_parameter('max_linear_vel', self.cfg.MAX_LINEAR_VEL)
        self.declare_parameter('max_angular_vel', self.cfg.MAX_ANGULAR_VEL)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('odom_publish_rate', self.cfg.ODOM_PUBLISH_RATE)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('invert_left_motor', False)
        self.declare_parameter('invert_right_motor', False)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('safety_stop_enabled', True)
        self.declare_parameter('stop_on_unknown', True)
        self.declare_parameter('obstacle_timeout', 1.0)
        self.declare_parameter('dry_run_log_period', 1.0)

    def _load_parameters(self):
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.obstacle_topic = self.get_parameter('obstacle_topic').value
        self.cfg.SERIAL_PORT = self.get_parameter('serial_port').value
        self.cfg.BAUD_RATE = int(self.get_parameter('baud_rate').value)
        self.cfg.DRIVER_ID = int(self.get_parameter('driver_id').value)
        self.cfg.WHEEL_RADIUS = float(
            self.get_parameter('wheel_radius').value
        )
        self.cfg.TRACK_WIDTH = float(
            self.get_parameter('track_width').value
        )
        self.cfg.GEAR_RATIO = float(
            self.get_parameter('gear_ratio').value
        )
        self.cfg.MAX_RPM = int(self.get_parameter('max_rpm').value)
        self.cfg.MAX_LINEAR_VEL = float(
            self.get_parameter('max_linear_vel').value
        )
        self.cfg.MAX_ANGULAR_VEL = float(
            self.get_parameter('max_angular_vel').value
        )
        self.cfg.ODOM_PUBLISH_RATE = float(
            self.get_parameter('odom_publish_rate').value
        )
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.invert_left_motor = bool(
            self.get_parameter('invert_left_motor').value
        )
        self.invert_right_motor = bool(
            self.get_parameter('invert_right_motor').value
        )
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.safety_stop_enabled = bool(
            self.get_parameter('safety_stop_enabled').value
        )
        self.stop_on_unknown = bool(
            self.get_parameter('stop_on_unknown').value
        )
        self.obstacle_timeout = float(
            self.get_parameter('obstacle_timeout').value
        )
        self.dry_run_log_period = float(
            self.get_parameter('dry_run_log_period').value
        )
        self.last_dry_run_log_time = self.get_clock().now()

    def obstacle_callback(self, msg):
        self.obstacle_decision = msg.decision
        self.obstacle_distance = msg.distance
        self.last_obstacle_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        left_rpm, right_rpm = self.kinematics.inverse_kinematics(
            msg.linear.x,
            msg.angular.z,
        )

        if self.invert_left_motor:
            left_rpm *= -1
        if self.invert_right_motor:
            right_rpm *= -1

        self.target_left_rpm = left_rpm
        self.target_right_rpm = right_rpm
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        elapsed = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.target_left_rpm = 0
            self.target_right_rpm = 0

        left_rpm, right_rpm = self.apply_safety_gate(
            self.target_left_rpm,
            self.target_right_rpm,
        )

        if self.connected:
            self.driver.send_rpm_command(left_rpm, right_rpm)
        elif self.dry_run:
            self.log_dry_run_command(left_rpm, right_rpm)

    def apply_safety_gate(self, left_rpm, right_rpm):
        if not self.safety_stop_enabled:
            self.safety_stop_active = False
            return left_rpm, right_rpm

        obstacle_fresh = self.is_obstacle_fresh()
        should_stop = False

        if not obstacle_fresh:
            should_stop = self.stop_on_unknown
        elif self.obstacle_decision == 'obstacle':
            should_stop = True
        elif self.obstacle_decision == 'unknown':
            should_stop = self.stop_on_unknown

        if should_stop and not self.safety_stop_active:
            self.get_logger().warn(
                'safety stop active: '
                f'decision={self.obstacle_decision}, '
                f'distance={self.obstacle_distance:.3f}'
            )
        elif not should_stop and self.safety_stop_active:
            self.get_logger().info('safety stop released')

        self.safety_stop_active = should_stop
        if should_stop:
            return 0, 0
        return left_rpm, right_rpm

    def is_obstacle_fresh(self):
        if self.last_obstacle_time is None:
            return False
        elapsed = (
            self.get_clock().now() - self.last_obstacle_time
        ).nanoseconds / 1e9
        return elapsed <= self.obstacle_timeout

    def log_dry_run_command(self, left_rpm, right_rpm):
        now = self.get_clock().now()
        elapsed = (now - self.last_dry_run_log_time).nanoseconds / 1e9
        if elapsed < self.dry_run_log_period:
            return

        self.get_logger().info(
            f'dry_run rpm left={left_rpm}, right={right_rpm}, '
            f'safety_stop={self.safety_stop_active}'
        )
        self.last_dry_run_log_time = now

    def publish_open_loop_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        if dt <= 0.0:
            return

        left_cmd, right_cmd = self.apply_safety_gate(
            self.target_left_rpm,
            self.target_right_rpm,
        )
        left_rpm = -left_cmd if self.invert_left_motor else left_cmd
        right_rpm = -right_cmd if self.invert_right_motor else right_cmd
        linear_v, angular_w = self.kinematics.forward_kinematics(
            left_rpm,
            right_rpm,
        )

        self.theta += angular_w * dt
        self.x += linear_v * math.cos(self.theta) * dt
        self.y += linear_v * math.sin(self.theta) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = linear_v
        odom.twist.twist.angular.z = angular_w
        self.odom_pub.publish(odom)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = odom.header.stamp
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        if self.driver is not None:
            self.driver.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
