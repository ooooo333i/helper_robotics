#!/usr/bin/env python3

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class BehaviorManagerNode(Node):
    """Translate perception behavior decisions into Nav2 commands."""

    VALID_BEHAVIORS = {'run', 'stop', 'overcome', 'avoid'}

    def __init__(self):
        super().__init__('behavior_manager_node')

        self.declare_parameter('behavior_cmd_topic', '/planning/behavior_cmd')
        self.declare_parameter('behavior_state_topic', '/planning/behavior_state')
        self.declare_parameter('goal_topic', '/planning/goal_pose')
        self.declare_parameter('speed_limit_topic', '/speed_limit')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter(
            'clear_local_costmap_service',
            '/local_costmap/clear_entirely_local_costmap',
        )
        self.declare_parameter(
            'clear_global_costmap_service',
            '/global_costmap/clear_entirely_global_costmap',
        )
        self.declare_parameter('overcome_speed_limit', 0.08)
        self.declare_parameter('behavior_publish_rate_hz', 2.0)

        self.behavior = 'run'
        self.current_goal = None
        self.goal_handle = None
        self.navigation_active = False
        self.goal_pending = False
        self.paused_by_behavior = False
        self.clear_in_progress = False

        behavior_cmd_topic = self.get_parameter('behavior_cmd_topic').value
        behavior_state_topic = self.get_parameter('behavior_state_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        speed_limit_topic = self.get_parameter('speed_limit_topic').value
        navigate_action = self.get_parameter('navigate_action').value

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            navigate_action,
        )
        self.local_clear_client = self.create_client(
            ClearEntireCostmap,
            self.get_parameter('clear_local_costmap_service').value,
        )
        self.global_clear_client = self.create_client(
            ClearEntireCostmap,
            self.get_parameter('clear_global_costmap_service').value,
        )

        self.create_subscription(
            String,
            behavior_cmd_topic,
            self.behavior_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            goal_topic,
            self.goal_callback,
            10,
        )
        self.behavior_pub = self.create_publisher(String, behavior_state_topic, 10)
        self.speed_limit_pub = self.create_publisher(
            SpeedLimit,
            speed_limit_topic,
            10,
        )
        self.create_timer(1.0, self.retry_pending_goal)
        behavior_rate = float(
            self.get_parameter('behavior_publish_rate_hz').value
        )
        self.create_timer(
            1.0 / max(behavior_rate, 0.1),
            self.publish_current_behavior_state,
        )

        self.get_logger().info(f'behavior cmd topic: {behavior_cmd_topic}')
        self.get_logger().info(f'behavior state topic: {behavior_state_topic}')
        self.get_logger().info(f'goal topic: {goal_topic}')
        self.get_logger().info(f'Nav2 action: {navigate_action}')

    def behavior_callback(self, msg):
        behavior = msg.data.strip().lower()
        if behavior not in self.VALID_BEHAVIORS:
            self.get_logger().warn(f'ignoring unknown behavior: {msg.data}')
            return
        if behavior == self.behavior:
            return

        previous = self.behavior
        self.behavior = behavior
        self.get_logger().info(f'behavior: {previous} -> {behavior}')
        self.publish_behavior_state(behavior)

        if behavior == 'run':
            self.clear_speed_limit()
            self.resume_current_goal()
        elif behavior == 'stop':
            self.stop_navigation()
        elif behavior == 'overcome':
            self.apply_overcome_speed_limit()
            self.resume_current_goal()
        elif behavior == 'avoid':
            self.clear_speed_limit()
            self.clear_costmaps()
            self.restart_current_goal()

    def goal_callback(self, msg):
        self.current_goal = msg
        self.get_logger().info(
            'received goal '
            f'frame={msg.header.frame_id} '
            f'x={msg.pose.position.x:.3f} '
            f'y={msg.pose.position.y:.3f}'
        )

        if self.behavior == 'stop':
            self.get_logger().info('goal stored while stopped; waiting for run')
            return

        self.send_goal(msg)

    def send_goal(self, pose):
        if not self.nav_client.server_is_ready():
            self.goal_pending = True
            self.get_logger().warn('Nav2 action server is not ready yet')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.nav_client.send_goal_async(goal)
        self.goal_pending = False
        self.paused_by_behavior = False
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.navigation_active = False
            self.get_logger().warn('Nav2 goal rejected')
            return

        self.navigation_active = True
        self.get_logger().info('Nav2 goal accepted')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        self.navigation_active = False
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 goal succeeded')
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Nav2 goal canceled')
        else:
            self.get_logger().warn(
                f'Nav2 goal ended with status={result.status}'
            )

    def stop_navigation(self):
        self.paused_by_behavior = True
        self.goal_pending = False
        if self.goal_handle is None or not self.navigation_active:
            return

        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        response = future.result()
        self.navigation_active = False
        if response.goals_canceling:
            self.get_logger().info('Nav2 cancel requested')
        else:
            self.get_logger().warn('Nav2 cancel request did not cancel a goal')

    def publish_behavior_state(self, behavior):
        msg = String()
        msg.data = behavior
        self.behavior_pub.publish(msg)

    def publish_current_behavior_state(self):
        self.publish_behavior_state(self.behavior)

    def resume_current_goal(self):
        if self.current_goal is None:
            return
        if self.navigation_active:
            return
        if not self.paused_by_behavior and not self.goal_pending:
            return
        self.send_goal(self.current_goal)

    def restart_current_goal(self):
        if self.current_goal is None:
            self.get_logger().warn('avoid requested, but no goal is stored')
            return

        self.goal_pending = True
        self.paused_by_behavior = False
        if self.goal_handle is not None and self.navigation_active:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.restart_after_cancel_callback)
            return

        self.send_goal(self.current_goal)

    def restart_after_cancel_callback(self, future):
        self.navigation_active = False
        if self.behavior != 'stop':
            self.send_goal(self.current_goal)

    def retry_pending_goal(self):
        if self.behavior == 'stop':
            return
        if not self.goal_pending:
            return
        if self.current_goal is None or self.navigation_active:
            return
        self.send_goal(self.current_goal)

    def clear_costmaps(self):
        if self.clear_in_progress:
            return

        self.clear_in_progress = True
        self.call_clear_costmap(self.local_clear_client, 'local')
        self.call_clear_costmap(self.global_clear_client, 'global')
        self.clear_in_progress = False

    def call_clear_costmap(self, client, label):
        if not client.service_is_ready():
            self.get_logger().warn(f'{label} costmap clear service is not ready')
            return

        future = client.call_async(ClearEntireCostmap.Request())
        future.add_done_callback(
            lambda done, name=label: self.clear_done_callback(done, name)
        )

    def clear_done_callback(self, future, label):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warn(f'{label} costmap clear failed: {exc}')
            return
        self.get_logger().info(f'{label} costmap cleared')

    def apply_overcome_speed_limit(self):
        limit = float(self.get_parameter('overcome_speed_limit').value)
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = max(limit, 0.0)
        self.speed_limit_pub.publish(msg)
        self.get_logger().info(f'overcome speed limit: {msg.speed_limit:.3f} m/s')

    def clear_speed_limit(self):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = 0.0
        self.speed_limit_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
