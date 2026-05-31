from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan_node',
            output='screen',
            parameters=[{
                'scan_time': 0.033,
                'range_min': 0.2,
                'range_max': 3.0,
                'scan_height': 3,
                'output_frame': 'camera_depth_optical_frame',
            }],
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/perception/scan/depth'),
            ],
        ),
    ])
