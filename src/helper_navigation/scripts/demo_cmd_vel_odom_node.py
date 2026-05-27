#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class DemoCmdVelOdomNode(Node):
    """Integrate cmd_vel into fake odometry for RViz/Nav2 demos."""

    def __init__(self):
        super().__init__('demo_cmd_vel_odom_node')

        self.declare_parameter('cmd_vel_topic', '/control/cmd_vel_safe')
        self.declare_parameter('odom_topic', '/control/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('max_linear_vel', 0.35)
        self.declare_parameter('max_angular_vel', 1.2)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_update_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_topic').value,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self.cmd_vel_callback,
            10,
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(rate, 1.0), self.timer_callback)

        self.get_logger().info(
            f'integrating {self.get_parameter("cmd_vel_topic").value} '
            f'to {self.get_parameter("odom_topic").value}'
        )

    def cmd_vel_callback(self, msg):
        max_linear = float(self.get_parameter('max_linear_vel').value)
        max_angular = float(self.get_parameter('max_angular_vel').value)

        self.linear_x = self.clamp(msg.linear.x, -max_linear, max_linear)
        self.angular_z = self.clamp(msg.angular.z, -max_angular, max_angular)
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        if dt <= 0.0:
            return

        elapsed_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        timeout = float(self.get_parameter('cmd_timeout_sec').value)
        if elapsed_cmd > timeout:
            linear_x = 0.0
            angular_z = 0.0
        else:
            linear_x = self.linear_x
            angular_z = self.angular_z

        self.theta = self.normalize_angle(self.theta + angular_z * dt)
        self.x += linear_x * math.cos(self.theta) * dt
        self.y += linear_x * math.sin(self.theta) * dt

        self.publish_odom(now, linear_x, angular_z)

    def publish_odom(self, stamp, linear_x, angular_z):
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value
        orientation_z = math.sin(self.theta / 2.0)
        orientation_w = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = orientation_z
        odom.pose.pose.orientation.w = orientation_w
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.angular.z = angular_z
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(min(float(value), maximum), minimum)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = DemoCmdVelOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
