import math

import rclpy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


class PerceptionBehaviorGateNode(Node):
    """Publish planning behavior commands from path and perception state."""

    def __init__(self):
        super().__init__('perception_behavior_gate_node')

        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('scan_topic', '/perception/scan/filtered')
        self.declare_parameter('odom_topic', '/control/odom')
        self.declare_parameter('behavior_cmd_topic', '/planning/behavior_cmd')
        self.declare_parameter(
            'dynamic_obstacle_topic',
            '/perception/obstacle/dynamic',
        )
        self.declare_parameter(
            'dynamic_speed_topic',
            '/perception/obstacle/dynamic_speed',
        )
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tracking_frame', 'odom')
        self.declare_parameter('path_timeout_sec', 1.0)
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('path_lookahead_m', 1.0) # 주행경로 기준 전방 1m
        self.declare_parameter('path_obstacle_width_m', 0.25)  # 주행경로 반경 0.25m
        self.declare_parameter('obstacle_min_range_m', 0.05)
        self.declare_parameter('obstacle_max_range_m', 2.0)
        self.declare_parameter('dynamic_speed_threshold_mps', 0.5)
        self.declare_parameter('dynamic_match_distance_m', 0.60)
        self.declare_parameter('cluster_max_gap_m', 0.15)
        self.declare_parameter('cluster_min_points', 3)
        self.declare_parameter('scan_sample_step', 1)
        self.declare_parameter('stop_latch_enabled', True)
        self.declare_parameter('stop_min_hold_sec', 0.8)
        self.declare_parameter('stop_clear_hold_sec', 2.0)
        self.declare_parameter('stopped_linear_threshold_mps', 0.03)
        self.declare_parameter('stopped_angular_threshold_radps', 0.08)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('publish_repeated_commands', False)

        self.path = None
        self.path_stamp = None
        self.scan = None
        self.scan_stamp = None
        self.odom = None
        self.odom_stamp = None
        self.last_behavior = None
        self.last_log_time = self.get_clock().now()
        self.last_obstacle_distance = math.inf
        self.last_dynamic_speed = math.inf
        self.tracked_obstacle = None
        self.dynamic_obstacle = False
        self.stop_latched = False
        self.stop_latch_time = None
        self.stop_clear_start_time = None
        self.stop_release_behavior = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.behavior_pub = self.create_publisher(
            String,
            self.get_parameter('behavior_cmd_topic').value,
            10,
        )
        self.dynamic_pub = self.create_publisher(
            Bool,
            self.get_parameter('dynamic_obstacle_topic').value,
            10,
        )
        self.dynamic_speed_pub = self.create_publisher(
            Float32,
            self.get_parameter('dynamic_speed_topic').value,
            10,
        )
        self.create_subscription(
            Path,
            self.get_parameter('path_topic').value,
            self.path_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(rate, 0.1), self.publish_behavior)

        self.get_logger().info(
            'perception behavior gate: '
            f'path={self.get_parameter("path_topic").value}, '
            f'scan={self.get_parameter("scan_topic").value}, '
            f'odom={self.get_parameter("odom_topic").value}, '
            f'behavior_cmd={self.get_parameter("behavior_cmd_topic").value}, '
            f'dynamic={self.get_parameter("dynamic_obstacle_topic").value}, '
            f'dynamic_speed={self.get_parameter("dynamic_speed_topic").value}'
        )

    def path_callback(self, msg):
        self.path = msg
        self.path_stamp = self.get_clock().now()

    def scan_callback(self, msg):
        self.scan = msg
        self.scan_stamp = self.get_clock().now()

    def odom_callback(self, msg):
        self.odom = msg
        self.odom_stamp = self.get_clock().now()

    def publish_behavior(self):
        if not self.path_is_fresh():
            self.publish_dynamic_status()
            self.log_status('waiting_for_path')
            return

        behavior = self.decide_behavior()
        self.publish_dynamic_status()
        if not self.should_publish(behavior):
            return

        msg = String()
        msg.data = behavior
        self.behavior_pub.publish(msg)
        self.last_behavior = behavior
        self.log_status(behavior)

    def publish_dynamic_status(self):
        dynamic_msg = Bool()
        dynamic_msg.data = bool(self.dynamic_obstacle)
        self.dynamic_pub.publish(dynamic_msg)

        speed_msg = Float32()
        speed_msg.data = (
            float(self.last_dynamic_speed)
            if math.isfinite(self.last_dynamic_speed)
            else -1.0
        )
        self.dynamic_speed_pub.publish(speed_msg)

    def decide_behavior(self):
        obstacle_on_path, distance, obstacle_center = (
            self.has_scan_obstacle_on_path()
        )
        self.last_obstacle_distance = distance
        raw_behavior = 'run'
        if obstacle_on_path:
            self.dynamic_obstacle = self.is_dynamic_obstacle(obstacle_center)
            if self.dynamic_obstacle:
                raw_behavior = 'stop'
            else:
                raw_behavior = 'avoid'
        else:
            self.tracked_obstacle = None
            self.dynamic_obstacle = False

        return self.apply_stop_latch(raw_behavior)

    def apply_stop_latch(self, raw_behavior):
        if not bool(self.get_parameter('stop_latch_enabled').value):
            return raw_behavior

        now = self.get_clock().now()
        if raw_behavior == 'stop':
            self.stop_latched = True
            if self.stop_latch_time is None:
                self.stop_latch_time = now
            self.stop_clear_start_time = None
            self.stop_release_behavior = None
            return 'stop'

        if not self.stop_latched:
            return raw_behavior

        hold_sec = float(self.get_parameter('stop_min_hold_sec').value)
        held_sec = (
            now - self.stop_latch_time
        ).nanoseconds / 1e9 if self.stop_latch_time is not None else 0.0
        if held_sec < hold_sec:
            return 'stop'
        if not self.stop_release_hold_satisfied(now, raw_behavior):
            return 'stop'

        self.stop_latched = False
        self.stop_latch_time = None
        self.stop_clear_start_time = None
        self.stop_release_behavior = None
        return raw_behavior

    def stop_release_hold_satisfied(self, now, raw_behavior):
        clear_hold_sec = float(
            self.get_parameter('stop_clear_hold_sec').value
        )
        if clear_hold_sec <= 0.0:
            return True

        if raw_behavior != self.stop_release_behavior:
            self.stop_release_behavior = raw_behavior
            self.stop_clear_start_time = now
            return False

        stable_sec = (
            now - self.stop_clear_start_time
        ).nanoseconds / 1e9
        return stable_sec >= clear_hold_sec

    def robot_is_stopped(self):
        if self.odom is None:
            return False

        linear = self.odom.twist.twist.linear
        angular = self.odom.twist.twist.angular
        linear_speed = math.hypot(linear.x, linear.y)
        angular_speed = abs(angular.z)
        linear_threshold = float(
            self.get_parameter('stopped_linear_threshold_mps').value
        )
        angular_threshold = float(
            self.get_parameter('stopped_angular_threshold_radps').value
        )
        return (
            linear_speed <= linear_threshold
            and angular_speed <= angular_threshold
        )

    def has_scan_obstacle_on_path(self):
        if not self.scan_is_fresh():
            return False, math.inf, None

        path_points = self.path_points_in_base_frame()
        if len(path_points) < 2:
            return False, math.inf, None

        clusters = self.scan_clusters_in_base_frame()
        if not clusters:
            return False, math.inf, None

        width = float(self.get_parameter('path_obstacle_width_m').value)
        closest = math.inf
        closest_cluster = None
        for cluster in clusters:
            distance = self.cluster_distance_to_path(cluster, path_points)
            if distance < closest:
                closest = distance
                closest_cluster = cluster

        if closest_cluster is not None and closest <= width:
            return True, closest, closest_cluster['centroid']

        centroid = closest_cluster['centroid'] if closest_cluster else None
        return False, closest, centroid

    def is_dynamic_obstacle(self, obstacle_point_base):
        point = self.point_in_tracking_frame(obstacle_point_base)
        if point is None:
            self.last_dynamic_speed = math.inf
            return False

        now = self.get_clock().now()
        if self.tracked_obstacle is None:
            self.tracked_obstacle = {
                'point': point,
                'time': now,
            }
            self.last_dynamic_speed = 0.0
            return False

        previous_point = self.tracked_obstacle['point']
        previous_time = self.tracked_obstacle['time']
        dt = (now - previous_time).nanoseconds / 1e9
        if dt <= 0.0:
            self.last_dynamic_speed = 0.0
            return False

        displacement = math.hypot(
            point[0] - previous_point[0],
            point[1] - previous_point[1],
        )
        match_distance = float(
            self.get_parameter('dynamic_match_distance_m').value
        )
        if displacement > match_distance:
            self.tracked_obstacle = {
                'point': point,
                'time': now,
            }
            self.last_dynamic_speed = math.inf
            return False

        speed = displacement / dt
        self.last_dynamic_speed = speed
        self.tracked_obstacle = {
            'point': point,
            'time': now,
        }
        threshold = float(
            self.get_parameter('dynamic_speed_threshold_mps').value
        )
        return speed >= threshold

    def point_in_tracking_frame(self, point_base):
        base_frame = self.get_parameter('base_frame').value
        tracking_frame = self.get_parameter('tracking_frame').value
        transform = None
        if base_frame != tracking_frame:
            transform = self.lookup_transform(tracking_frame, base_frame)
        if base_frame != tracking_frame and transform is None:
            return None
        return self.transform_xy(point_base[0], point_base[1], transform)

    def path_points_in_base_frame(self):
        base_frame = self.get_parameter('base_frame').value
        path_frame = self.path.header.frame_id
        if not path_frame:
            path_frame = base_frame

        transform = None
        if path_frame != base_frame:
            transform = self.lookup_transform(base_frame, path_frame)
        if path_frame != base_frame and transform is None:
            return []

        lookahead = float(self.get_parameter('path_lookahead_m').value)
        points = []
        for pose in self.path.poses:
            x, y = self.transform_xy(
                pose.pose.position.x,
                pose.pose.position.y,
                transform,
            )
            if x < -0.05:
                continue
            if math.hypot(x, y) > lookahead:
                continue
            points.append((x, y))
        return points

    def scan_clusters_in_base_frame(self):
        base_frame = self.get_parameter('base_frame').value
        scan_frame = self.scan.header.frame_id
        if not scan_frame:
            scan_frame = base_frame

        transform = None
        if scan_frame != base_frame:
            transform = self.lookup_transform(base_frame, scan_frame)
        if scan_frame != base_frame and transform is None:
            return []

        lookahead = float(self.get_parameter('path_lookahead_m').value)
        min_range = float(self.get_parameter('obstacle_min_range_m').value)
        max_range = float(self.get_parameter('obstacle_max_range_m').value)
        step = max(int(self.get_parameter('scan_sample_step').value), 1)

        points = []
        angle = self.scan.angle_min
        for index, distance in enumerate(self.scan.ranges):
            if index % step != 0:
                angle += self.scan.angle_increment
                continue

            if math.isfinite(distance) and min_range <= distance <= max_range:
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                x, y = self.transform_xy(x, y, transform)
                if x >= 0.0 and math.hypot(x, y) <= lookahead:
                    points.append((x, y))

            angle += self.scan.angle_increment
        return self.cluster_scan_points(points)

    def cluster_scan_points(self, points):
        max_gap = float(self.get_parameter('cluster_max_gap_m').value)
        min_points = int(self.get_parameter('cluster_min_points').value)

        clusters = []
        current = []
        previous = None
        for point in points:
            if previous is None:
                current = [point]
            elif math.hypot(
                point[0] - previous[0],
                point[1] - previous[1],
            ) <= max_gap:
                current.append(point)
            else:
                self.append_cluster(clusters, current, min_points)
                current = [point]
            previous = point

        self.append_cluster(clusters, current, min_points)
        return clusters

    def append_cluster(self, clusters, points, min_points):
        if len(points) < min_points:
            return

        count = len(points)
        centroid_x = sum(point[0] for point in points) / count
        centroid_y = sum(point[1] for point in points) / count
        min_range = min(math.hypot(point[0], point[1]) for point in points)
        clusters.append({
            'points': points,
            'centroid': (centroid_x, centroid_y),
            'min_range': min_range,
            'point_count': count,
        })

    def cluster_distance_to_path(self, cluster, path_points):
        return min(
            self.distance_to_path(point, path_points)
            for point in cluster['points']
        )

    def lookup_transform(self, target_frame, source_frame):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
            ).transform
        except TransformException as exc:
            self.get_logger().warn(
                f'transform unavailable {source_frame}->{target_frame}: {exc}',
                throttle_duration_sec=1.0,
            )
            return None

    def transform_xy(self, x, y, transform):
        if transform is None:
            return x, y

        yaw = self.yaw_from_quaternion(transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        out_x = cos_yaw * x - sin_yaw * y + transform.translation.x
        out_y = sin_yaw * x + cos_yaw * y + transform.translation.y
        return out_x, out_y

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def distance_to_path(self, point, path_points):
        closest = math.inf
        for start, end in zip(path_points[:-1], path_points[1:]):
            closest = min(
                closest,
                self.distance_to_segment(point, start, end),
            )
        return closest

    @staticmethod
    def distance_to_segment(point, start, end):
        px, py = point
        sx, sy = start
        ex, ey = end
        vx = ex - sx
        vy = ey - sy
        wx = px - sx
        wy = py - sy

        length_sq = vx * vx + vy * vy
        if length_sq <= 1e-9:
            return math.hypot(px - sx, py - sy)

        t = (wx * vx + wy * vy) / length_sq
        t = min(max(t, 0.0), 1.0)
        proj_x = sx + t * vx
        proj_y = sy + t * vy
        return math.hypot(px - proj_x, py - proj_y)

    def path_is_fresh(self):
        if self.path is None or self.path_stamp is None:
            return False
        timeout_sec = float(self.get_parameter('path_timeout_sec').value)
        elapsed = (
            self.get_clock().now() - self.path_stamp
        ).nanoseconds / 1e9
        return elapsed <= timeout_sec

    def scan_is_fresh(self):
        if self.scan is None or self.scan_stamp is None:
            return False
        timeout_sec = float(self.get_parameter('scan_timeout_sec').value)
        elapsed = (
            self.get_clock().now() - self.scan_stamp
        ).nanoseconds / 1e9
        return elapsed <= timeout_sec

    def should_publish(self, behavior):
        if bool(self.get_parameter('publish_repeated_commands').value):
            return True
        return behavior != self.last_behavior

    def log_status(self, behavior):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < 1_000_000_000:
            return
        self.last_log_time = now

        path_size = len(self.path.poses) if self.path is not None else 0
        obstacle_text = (
            f'{self.last_obstacle_distance:.3f}m'
            if math.isfinite(self.last_obstacle_distance)
            else 'none'
        )
        speed_text = (
            f'{self.last_dynamic_speed:.3f}m/s'
            if math.isfinite(self.last_dynamic_speed)
            else 'none'
        )
        self.get_logger().info(
            f'behavior={behavior}, path_poses={path_size}, '
            f'path_obstacle_distance={obstacle_text}, '
            f'dynamic_speed={speed_text}, '
            f'dynamic_obstacle={self.dynamic_obstacle}, '
            f'stop_latched={self.stop_latched}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionBehaviorGateNode()
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
