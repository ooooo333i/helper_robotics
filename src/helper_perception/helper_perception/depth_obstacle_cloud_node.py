import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField


class DepthObstacleCloudNode(Node):
    """Publish costmap obstacle points from filtered depth-image pixels."""

    def __init__(self):
        super().__init__('depth_obstacle_cloud_node')

        self.declare_parameter(
            'input_depth_topic',
            '/camera/camera/depth/image_rect_raw',
        )
        self.declare_parameter(
            'input_camera_info_topic',
            '/camera/camera/depth/camera_info',
        )
        self.declare_parameter(
            'output_points_topic',
            '/perception/depth/obstacle_points',
        )
        self.declare_parameter(
            'output_clearing_points_topic',
            '/perception/depth/clearing_points',
        )
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('roi_x_min_ratio', 0.35)
        self.declare_parameter('roi_x_max_ratio', 0.65)
        self.declare_parameter('roi_y_min_ratio', 0.35)
        self.declare_parameter('roi_y_max_ratio', 0.75)
        self.declare_parameter('min_valid_depth', 0.2)
        self.declare_parameter('max_valid_depth', 2.0)
        self.declare_parameter('depth_unit_scale', 0.001)
        self.declare_parameter('min_obstacle_height_m', 0.10)
        self.declare_parameter('max_obstacle_height_m', 0.30)
        self.declare_parameter('camera_height_m', 0.1889)
        self.declare_parameter('camera_pitch_deg', 55.0)
        self.declare_parameter('camera_x_offset_m', 0.1847)
        self.declare_parameter('sample_step', 4)
        self.declare_parameter('max_points', 3000)

        self.camera_info = None
        self.obstacle_publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter('output_points_topic').value,
            10,
        )
        self.clearing_publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter('output_clearing_points_topic').value,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter('input_depth_topic').value,
            self.depth_callback,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter('input_camera_info_topic').value,
            self.camera_info_callback,
            10,
        )
        self.last_log_time = self.get_clock().now()

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        obstacle_points, clearing_points = self.depth_to_cloud_points(msg)
        obstacle_cloud = self.make_cloud(
            msg.header.stamp,
            obstacle_points,
            self.get_parameter('output_frame').value,
        )
        clearing_cloud = self.make_cloud(
            msg.header.stamp,
            clearing_points,
            self.clearing_frame_id(msg),
        )
        self.obstacle_publisher.publish(obstacle_cloud)
        self.clearing_publisher.publish(clearing_cloud)
        self.log_status(len(obstacle_points), len(clearing_points))

    def clearing_frame_id(self, depth_msg):
        if self.camera_info is not None and self.camera_info.header.frame_id:
            return self.camera_info.header.frame_id
        if depth_msg.header.frame_id:
            return depth_msg.header.frame_id

        self.get_logger().warn(
            'Depth image and CameraInfo have no frame_id; '
            'using camera_depth_optical_frame.',
            throttle_duration_sec=2.0,
        )
        return 'camera_depth_optical_frame'

    def depth_to_cloud_points(self, msg):
        empty = np.empty((0, 3), dtype=np.float32)
        if self.camera_info is None:
            return empty, empty

        depth = self.image_to_depth_meters(msg)
        if depth is None:
            return empty, empty

        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        if fx == 0.0 or fy == 0.0:
            return empty, empty

        image_height, image_width = depth.shape
        x_min, x_max, y_min, y_max = self.roi_bounds(
            image_width,
            image_height,
        )
        step = max(int(self.get_parameter('sample_step').value), 1)
        roi = depth[y_min:y_max:step, x_min:x_max:step]

        valid = np.isfinite(roi)
        valid &= roi >= float(self.get_parameter('min_valid_depth').value)
        valid &= roi <= float(self.get_parameter('max_valid_depth').value)
        if not np.any(valid):
            return empty, empty

        ys, xs = np.nonzero(valid)
        z = roi[ys, xs].astype(np.float32)
        pixel_x = xs.astype(np.float32) * step + float(x_min)
        pixel_y = ys.astype(np.float32) * step + float(y_min)

        camera_x_right = (pixel_x - cx) * z / fx
        camera_y_down = (pixel_y - cy) * z / fy
        clearing_points = np.column_stack((
            camera_x_right,
            camera_y_down,
            z,
        )).astype(np.float32)

        pitch = math.radians(self.get_parameter('camera_pitch_deg').value)
        camera_height = float(self.get_parameter('camera_height_m').value)
        vertical_down = z * math.sin(pitch) + camera_y_down * math.cos(pitch)
        ground_forward = z * math.cos(pitch) - camera_y_down * math.sin(pitch)
        height_above_floor = camera_height - vertical_down

        min_height = float(self.get_parameter('min_obstacle_height_m').value)
        max_height = float(self.get_parameter('max_obstacle_height_m').value)
        obstacle = np.isfinite(height_above_floor)
        obstacle &= height_above_floor >= min_height
        obstacle &= height_above_floor <= max_height
        obstacle &= ground_forward > 0.0
        if not np.any(obstacle):
            return empty, self.limit_points(clearing_points)

        camera_x_offset = float(
            self.get_parameter('camera_x_offset_m').value
        )
        obstacle_points = np.column_stack((
            ground_forward[obstacle] + camera_x_offset,
            -camera_x_right[obstacle],
            height_above_floor[obstacle],
        )).astype(np.float32)

        return (
            self.limit_points(obstacle_points),
            self.limit_points(clearing_points),
        )

    def limit_points(self, points):
        max_points = max(int(self.get_parameter('max_points').value), 1)
        if points.shape[0] <= max_points:
            return points

        indices = np.linspace(
            0,
            points.shape[0] - 1,
            max_points,
            dtype=np.int32,
        )
        return points[indices]

    def image_to_depth_meters(self, msg):
        encoding = msg.encoding.upper()
        scale = float(self.get_parameter('depth_unit_scale').value)

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
        return depth * scale

    def roi_bounds(self, width, height):
        x_min_ratio = self.get_parameter('roi_x_min_ratio').value
        x_max_ratio = self.get_parameter('roi_x_max_ratio').value
        y_min_ratio = self.get_parameter('roi_y_min_ratio').value
        y_max_ratio = self.get_parameter('roi_y_max_ratio').value

        x_min = int(width * min(max(x_min_ratio, 0.0), 1.0))
        x_max = int(width * min(max(x_max_ratio, 0.0), 1.0))
        y_min = int(height * min(max(y_min_ratio, 0.0), 1.0))
        y_max = int(height * min(max(y_max_ratio, 0.0), 1.0))

        if x_max <= x_min:
            x_min, x_max = 0, width
        if y_max <= y_min:
            y_min, y_max = 0, height
        return x_min, x_max, y_min, y_max

    def make_cloud(self, stamp, points, frame_id):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def log_status(self, obstacle_count, clearing_count):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < 1_000_000_000:
            return
        self.last_log_time = now
        self.get_logger().info(
            f'depth cloud obstacle_points={obstacle_count}, '
            f'clearing_points={clearing_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleCloudNode()
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
