# 외부 LiDAR driver를 rear 센서용으로 실행하는 launch 파일

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rear_serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('rear_serial_baudrate', default_value='460800'),
        DeclareLaunchArgument('rear_frame_id', default_value='laser_rear'),
        DeclareLaunchArgument('rear_inverted', default_value='false'),
        DeclareLaunchArgument('rear_angle_compensate', default_value='true'),
        DeclareLaunchArgument('rear_scan_mode', default_value='Standard'),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rear_sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('rear_serial_port'),
                'serial_baudrate': ParameterValue(
                    LaunchConfiguration('rear_serial_baudrate'),
                    value_type=int,
                ),
                'frame_id': LaunchConfiguration('rear_frame_id'),
                'inverted': ParameterValue(
                    LaunchConfiguration('rear_inverted'),
                    value_type=bool,
                ),
                'angle_compensate': ParameterValue(
                    LaunchConfiguration('rear_angle_compensate'),
                    value_type=bool,
                ),
                'scan_mode': LaunchConfiguration('rear_scan_mode'),
            }],
            remappings=[
                ('scan', '/perception/lidar/rear/scan_raw'),
            ],
        ),
    ])
