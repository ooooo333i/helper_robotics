# 외부에 LiDAR driver 실행하도록 하는 launch 파일임

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('front_serial_port', default_value='/dev/lidar_front'),  # port
        DeclareLaunchArgument('front_serial_baudrate', default_value='460800'),  # 통신속도
        DeclareLaunchArgument('front_frame_id', default_value='laser_front'),  # 좌표계 기준
        DeclareLaunchArgument('front_inverted', default_value='false'),  # 반대로 장착할 경우 true
        DeclareLaunchArgument('front_angle_compensate', default_value='true'),  # 각도 보정
        DeclareLaunchArgument('front_scan_mode', default_value='Standard'),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',  # ros2 run sllidar_ros2 sllidar_node 역할
            name='front_sllidar_node',  # node 이름
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('front_serial_port'),
                'serial_baudrate': ParameterValue(
                    LaunchConfiguration('front_serial_baudrate'),
                    value_type=int,
                ),
                'frame_id': LaunchConfiguration('front_frame_id'),
                'inverted': ParameterValue(
                    LaunchConfiguration('front_inverted'),
                    value_type=bool,
                ),
                'angle_compensate': ParameterValue(
                    LaunchConfiguration('front_angle_compensate'),
                    value_type=bool,
                ),
                'scan_mode': LaunchConfiguration('front_scan_mode'),
            }],
            # The driver publishes relative topic "scan"; expose it as our raw front scan.
            remappings=[
                ('scan', '/perception/lidar/front/scan_raw'),
            ],
        ),
    ])
