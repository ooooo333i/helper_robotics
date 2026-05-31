import math

import rclpy
from helper_msgs.msg import ObstacleDecision
from rclpy.node import Node
from std_msgs.msg import String


class ObstacleActionNode(Node):
    """Decide stop / go / turn from LiDAR and depth-camera obstacle topics."""

    def __init__(self):
        super().__init__('obstacle_action_node')

        # ── 입력 토픽 ──────────────────────────────────────────────
        self.declare_parameter('lidar_topic', '/perception/obstacle/range')
        self.declare_parameter('depth_topic', '/perception/obstacle/depth')
        self.declare_parameter('output_topic', '/perception/obstacle/action')

        # ── 라이다 임계값 ───────────────────────────────────────────
        # lidar_stop_distance_m  : 이 거리보다 가까우면 무조건 정지
        # lidar_turn_distance_m  : 이 거리보다 가까우면 선회 권고
        self.declare_parameter('lidar_stop_distance_m', 0.40)
        self.declare_parameter('lidar_turn_distance_m', 0.80)

        # ── 뎁스카메라 임계값 ───────────────────────────────────────
        # depth_stop_distance_m  : 이 거리보다 가까우면 정지
        # depth_turn_distance_m  : 이 거리보다 가까우면 선회 권고
        # depth_min_height_m     : 이 높이보다 낮은 장애물은 무시 (바닥 노이즈 제거)
        # depth_max_height_m     : 이 높이보다 높은 장애물은 무시 (로봇 위로 지나가는 구조물 등)
        self.declare_parameter('depth_stop_distance_m', 0.50)
        self.declare_parameter('depth_turn_distance_m', 1.00)
        self.declare_parameter('depth_min_height_m', 0.05)
        self.declare_parameter('depth_max_height_m', 0.60)

        # ── 타임아웃 ────────────────────────────────────────────────
        # input_timeout_sec  : 이 시간 내에 메시지가 없으면 stale 처리
        # timeout_action     : stale 입력 발생 시 출력 ("stop" 또는 "go")
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('timeout_action', 'stop')

        # ── 발행 주기 ────────────────────────────────────────────────
        self.declare_parameter('publish_rate_hz', 10.0)

        self._lidar_msg: ObstacleDecision | None = None
        self._depth_msg: ObstacleDecision | None = None
        self._lidar_stamp = None
        self._depth_stamp = None

        self._pub = self.create_publisher(
            String,
            self.get_parameter('output_topic').value,
            10,
        )
        self.create_subscription(
            ObstacleDecision,
            self.get_parameter('lidar_topic').value,
            self._lidar_cb,
            10,
        )
        self.create_subscription(
            ObstacleDecision,
            self.get_parameter('depth_topic').value,
            self._depth_cb,
            10,
        )

        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / max(rate, 1.0), self._publish)
        self._last_log_time = self.get_clock().now()

    # ── 콜백 ────────────────────────────────────────────────────────

    def _lidar_cb(self, msg: ObstacleDecision):
        self._lidar_msg = msg
        self._lidar_stamp = self.get_clock().now()

    def _depth_cb(self, msg: ObstacleDecision):
        self._depth_msg = msg
        self._depth_stamp = self.get_clock().now()

    # ── 발행 타이머 ─────────────────────────────────────────────────

    def _publish(self):
        now = self.get_clock().now()
        lidar_fresh = self._is_fresh(self._lidar_stamp, now)
        depth_fresh = self._is_fresh(self._depth_stamp, now)

        action = self._decide(lidar_fresh, depth_fresh)

        out = String()
        out.data = action
        self._pub.publish(out)

        self._log(action, lidar_fresh, depth_fresh, now)

    # ── 결정 로직 ────────────────────────────────────────────────────

    def _decide(self, lidar_fresh: bool, depth_fresh: bool) -> str:
        # 두 센서 모두 stale → timeout_action 반환
        if not lidar_fresh and not depth_fresh:
            return self.get_parameter('timeout_action').value

        lidar_stop = self.get_parameter('lidar_stop_distance_m').value
        lidar_turn = self.get_parameter('lidar_turn_distance_m').value
        depth_stop = self.get_parameter('depth_stop_distance_m').value
        depth_turn = self.get_parameter('depth_turn_distance_m').value
        depth_min_h = self.get_parameter('depth_min_height_m').value
        depth_max_h = self.get_parameter('depth_max_height_m').value

        lidar_dist, lidar_blocked = self._eval_lidar(lidar_fresh)
        depth_dist, depth_blocked = self._eval_depth(depth_fresh, depth_min_h, depth_max_h)

        # 정지 우선 판단
        if lidar_blocked and lidar_dist <= lidar_stop:
            return 'stop'
        if depth_blocked and depth_dist <= depth_stop:
            return 'stop'

        # 선회 판단
        if lidar_blocked and lidar_dist <= lidar_turn:
            return 'turn'
        if depth_blocked and depth_dist <= depth_turn:
            return 'turn'

        return 'go'

    def _eval_lidar(self, fresh: bool):
        """라이다 장애물 여부와 거리 반환."""
        if not fresh or self._lidar_msg is None:
            return math.inf, False
        dist = self._lidar_msg.distance
        if not math.isfinite(dist):
            dist = math.inf
        blocked = self._lidar_msg.decision == 'obstacle'
        return dist, blocked

    def _eval_depth(self, fresh: bool, min_h: float, max_h: float):
        """뎁스카메라 장애물 여부(높이 필터 적용)와 거리 반환."""
        if not fresh or self._depth_msg is None:
            return math.inf, False
        dist = self._depth_msg.distance
        if not math.isfinite(dist):
            dist = math.inf
        height = self._depth_msg.height
        height_valid = min_h <= height <= max_h
        blocked = self._depth_msg.decision == 'obstacle' and height_valid
        return dist, blocked

    # ── 유틸 ────────────────────────────────────────────────────────

    def _is_fresh(self, stamp, now) -> bool:
        if stamp is None:
            return False
        timeout_ns = int(self.get_parameter('input_timeout_sec').value * 1e9)
        return (now - stamp).nanoseconds <= timeout_ns

    def _log(self, action: str, lidar_fresh: bool, depth_fresh: bool, now):
        if (now - self._last_log_time).nanoseconds < 1_000_000_000:
            return
        self._last_log_time = now

        def dist_str(msg, fresh):
            if not fresh or msg is None:
                return 'stale'
            d = msg.distance
            return f'{d:.2f}m' if math.isfinite(d) else 'inf'

        lidar_s = dist_str(self._lidar_msg, lidar_fresh)
        depth_s = dist_str(self._depth_msg, depth_fresh)
        h_s = (
            f'{self._depth_msg.height:.2f}m'
            if depth_fresh and self._depth_msg is not None
            else '-'
        )
        self.get_logger().info(
            f'action={action}  '
            f'lidar={lidar_s}  '
            f'depth={depth_s}(h={h_s})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
