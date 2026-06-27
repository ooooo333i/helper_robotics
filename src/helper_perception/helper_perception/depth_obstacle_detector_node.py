import math

import numpy as np
import rclpy
from helper_msgs.msg import ObstacleDecision
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class DepthObstacleDetectorNode(Node):
    """Detect nearby obstacles from a configurable depth-image ROI."""

    def __init__(self):
        super().__init__('depth_obstacle_detector_node')

        self.declare_parameter(
            'input_depth_topic',
            '/perception/depth/image_raw',
        )
        self.declare_parameter(
            'input_camera_info_topic',
            '/perception/depth/camera_info',
        )
        self.declare_parameter(
            'output_obstacle_topic',
            '/perception/obstacle/depth',
        )
        self.declare_parameter('roi_apex_x_ratio', 0.5)
        self.declare_parameter('roi_apex_y_ratio', 0.5)
        self.declare_parameter('obstacle_distance_threshold', 0.8)
        self.declare_parameter('min_valid_depth', 0.2)
        self.declare_parameter('max_valid_depth', 3.0)
        self.declare_parameter('depth_unit_scale', 0.001)
        self.declare_parameter('distance_percentile', 10.0)
        self.declare_parameter('camera_height_m', 0.27)
        self.declare_parameter('camera_pitch_deg', 45.0)
        self.declare_parameter('height_depth_window_m', 0.08)
        self.declare_parameter('debug_topic', '/perception/obstacle/depth_debug')

        input_depth_topic = self.get_parameter('input_depth_topic').value
        input_camera_info_topic = self.get_parameter(
            'input_camera_info_topic'
        ).value
        output_obstacle_topic = self.get_parameter(
            'output_obstacle_topic'
        ).value

        self.camera_info = None
        self.publisher = self.create_publisher(
            ObstacleDecision,
            output_obstacle_topic,
            10,
        )
        self.debug_publisher = self.create_publisher(
            String,
            self.get_parameter('debug_topic').value,
            10,
        )
        self.subscription = self.create_subscription(
            Image,
            input_depth_topic,
            self.depth_callback,
            10,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            input_camera_info_topic,
            self.camera_info_callback,
            10,
        )
        self.last_log_time = self.get_clock().now()

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        depth = self.image_to_depth_meters(msg)
        if depth is None:
            self.publish_decision('unknown', math.inf, 0.0)
            return

        mask = self.get_roi_mask(*depth.shape[::-1])
        roi = depth[mask]
        min_valid_depth = self.get_parameter('min_valid_depth').value
        max_valid_depth = self.get_parameter('max_valid_depth').value
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid >= min_valid_depth) & (valid <= max_valid_depth)]

        if valid.size == 0:
            raw_depth = math.inf
            distance = math.inf
            decision = 'unknown'
            height = 0.0
            debug = self.make_debug_text(
                decision,
                raw_depth,
                distance,
                height,
                valid.size,
                None,
            )
        else:
            percentile = self.get_parameter('distance_percentile').value
            percentile = min(100.0, max(0.0, percentile))
            raw_depth = float(np.percentile(valid, percentile))
            height, ground_distance = self.estimate_obstacle_geometry(
                depth,
                raw_depth,
            )
            distance = (
                ground_distance
                if math.isfinite(ground_distance)
                else raw_depth
            )
            threshold = self.get_parameter('obstacle_distance_threshold').value
            decision = 'obstacle' if distance <= threshold else 'clear'
            debug = self.make_debug_text(
                decision,
                raw_depth,
                distance,
                height,
                valid.size,
                valid,
            )

        self.publish_decision(decision, distance, height)
        self.publish_debug(debug)
        self.log_status(decision, distance, height, debug)

    def image_to_depth_meters(self, msg):
        encoding = msg.encoding.upper()
        scale = self.get_parameter('depth_unit_scale').value

        if encoding == '16UC1':
            dtype = np.uint16
        elif encoding == '32FC1':
            dtype = np.float32
        else:
            self.get_logger().warn(
                f'Unsupported depth encoding: {msg.encoding}',
                throttle_duration_sec=2.0,
            )
            return None

        expected_step = msg.width * np.dtype(dtype).itemsize
        if msg.step < expected_step:
            self.get_logger().warn('Depth image step is smaller than expected')
            return None

        raw = np.frombuffer(msg.data, dtype=dtype)
        rows = raw.reshape((msg.height, msg.step // np.dtype(dtype).itemsize))
        depth = rows[:, :msg.width].astype(np.float32)
        return depth * float(scale)

    def get_roi_mask(self, width, height):
        """Triangular ROI: bottom-left and bottom-right corners, apex at image center (by default)."""
        apex_x_ratio = self.get_parameter('roi_apex_x_ratio').value
        apex_y_ratio = self.get_parameter('roi_apex_y_ratio').value
        apex = (
            width * min(max(apex_x_ratio, 0.0), 1.0),
            height * min(max(apex_y_ratio, 0.0), 1.0),
        )
        bottom_left = (0.0, float(height - 1))
        bottom_right = (float(width - 1), float(height - 1))

        ys, xs = np.mgrid[0:height, 0:width]
        return self.points_in_triangle(xs, ys, bottom_left, bottom_right, apex)

    @staticmethod
    def points_in_triangle(px, py, a, b, c):
        def sign(x1, y1, x2, y2, x3, y3):
            return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3)

        d1 = sign(px, py, a[0], a[1], b[0], b[1])
        d2 = sign(px, py, b[0], b[1], c[0], c[1])
        d3 = sign(px, py, c[0], c[1], a[0], a[1])

        has_neg = (d1 < 0) | (d2 < 0) | (d3 < 0)
        has_pos = (d1 > 0) | (d2 > 0) | (d3 > 0)
        return ~(has_neg & has_pos)

    def estimate_obstacle_geometry(self, depth, distance):
        if self.camera_info is None or not math.isfinite(distance):
            return 0.0, math.inf

        fy = float(self.camera_info.k[4])
        cy = float(self.camera_info.k[5])
        if fy == 0.0:
            return 0.0, math.inf

        image_height, image_width = depth.shape
        mask = self.get_roi_mask(image_width, image_height)
        min_valid_depth = self.get_parameter('min_valid_depth').value
        max_valid_depth = self.get_parameter('max_valid_depth').value
        depth_window = self.get_parameter('height_depth_window_m').value

        valid = mask & np.isfinite(depth)
        valid &= depth >= min_valid_depth
        valid &= depth <= max_valid_depth
        valid &= depth <= distance + depth_window
        if not np.any(valid):
            return 0.0, math.inf

        ys, xs = np.nonzero(valid)
        z = depth[ys, xs].astype(np.float32)
        pixel_y = ys.astype(np.float32)
        camera_y_down = (pixel_y - cy) * z / fy

        camera_height = self.get_parameter('camera_height_m').value
        pitch = math.radians(self.get_parameter('camera_pitch_deg').value)
        vertical_down = z * math.sin(pitch) + camera_y_down * math.cos(pitch)
        ground_forward = z * math.cos(pitch) - camera_y_down * math.sin(pitch)
        height_above_floor = float(camera_height) - vertical_down

        ground_forward = ground_forward[np.isfinite(ground_forward)]
        ground_forward = ground_forward[ground_forward > 0.0]
        ground_distance = (
            float(np.percentile(ground_forward, 10.0))
            if ground_forward.size > 0
            else math.inf
        )

        height_above_floor = height_above_floor[np.isfinite(height_above_floor)]
        height_above_floor = height_above_floor[height_above_floor > 0.0]
        if height_above_floor.size == 0:
            return 0.0, ground_distance

        return float(np.percentile(height_above_floor, 95.0)), ground_distance

    def make_debug_text(
        self,
        decision,
        raw_depth,
        distance,
        height,
        valid_count,
        valid_depths,
    ):
        parts = [
            f'decision={decision}',
            f'distance_m={self.format_float(distance)}',
            f'raw_depth_m={self.format_float(raw_depth)}',
            f'height_m={self.format_float(height)}',
            f'valid_roi_pixels={valid_count}',
            f'camera_height_m={self.get_parameter("camera_height_m").value:.3f}',
            f'camera_pitch_deg={self.get_parameter("camera_pitch_deg").value:.1f}',
        ]
        if valid_depths is not None and valid_depths.size > 0:
            parts.extend([
                f'roi_min_m={float(np.min(valid_depths)):.3f}',
                f'roi_p10_m={float(np.percentile(valid_depths, 10.0)):.3f}',
                f'roi_median_m={float(np.median(valid_depths)):.3f}',
            ])
        return ', '.join(parts)

    def format_float(self, value):
        return f'{value:.3f}' if math.isfinite(value) else 'inf'

    def publish_decision(self, decision, distance, height):
        msg = ObstacleDecision()
        msg.obstacle_type = 'depth'
        msg.decision = decision
        msg.distance = float(distance)
        msg.height = float(height)
        msg.is_dynamic = False
        self.publisher.publish(msg)

    def publish_debug(self, debug):
        msg = String()
        msg.data = debug
        self.debug_publisher.publish(msg)

    def log_status(self, decision, distance, height, debug):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < 1_000_000_000:
            return

        distance_text = (
            f'{distance:.3f} m' if math.isfinite(distance) else 'inf'
        )
        height_text = f'{height:.3f} m'
        self.get_logger().info(
            f'depth_distance={distance_text}, height={height_text}, '
            f'decision={decision}, debug=({debug})'
        )
        self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleDetectorNode()
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
