# LiDAR raw scan을 인식용 데이터 가공
# 장애물 유무 판단하는 perception node 실행 파일

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_filter_node_name',
            default_value='scan_filter_node',
        ),
        DeclareLaunchArgument(
            'obstacle_detector_node_name',
            default_value='obstacle_detector_node',
        ),
        DeclareLaunchArgument(
            'input_scan_topic',
            default_value='/perception/scan/front/raw',
        ),
        DeclareLaunchArgument(
            'filtered_scan_topic',
            default_value='/perception/scan/front',
        ),
        DeclareLaunchArgument(
            'obstacle_topic',
            default_value='/perception/obstacle/front',
        ),
        DeclareLaunchArgument('angle_min_deg', default_value='-90.0'), # LiDAR 최소 각도 (오)
        DeclareLaunchArgument('angle_max_deg', default_value='90.0'),
        DeclareLaunchArgument('min_valid_range', default_value='0.15'), # 유효 최소 거리 (m단위)
        DeclareLaunchArgument('max_valid_range', default_value='8.0'),
        DeclareLaunchArgument('detection_angle_min_deg', default_value='-30.0'), # 실제 장애물 판단 최소 각도
        DeclareLaunchArgument('detection_angle_max_deg', default_value='30.0'),
        DeclareLaunchArgument('obstacle_distance_threshold', default_value='0.5'), # 장애물로 판단할 거리 기준
        Node(
            package='helper_perception',
            executable='scan_filter_node',
            name=LaunchConfiguration('scan_filter_node_name'),
            output='screen',
            parameters=[{
                'input_scan_topic': LaunchConfiguration('input_scan_topic'),
                'output_scan_topic': LaunchConfiguration('filtered_scan_topic'),
                'angle_min_deg': ParameterValue(
                    LaunchConfiguration('angle_min_deg'),
                    value_type=float,
                ),
                'angle_max_deg': ParameterValue(
                    LaunchConfiguration('angle_max_deg'),
                    value_type=float,
                ),
                'min_valid_range': ParameterValue(
                    LaunchConfiguration('min_valid_range'),
                    value_type=float,
                ),
                'max_valid_range': ParameterValue(
                    LaunchConfiguration('max_valid_range'),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package='helper_perception',
            executable='obstacle_detector_node',
            name=LaunchConfiguration('obstacle_detector_node_name'),
            output='screen',
            parameters=[{
                'input_scan_topic': LaunchConfiguration('filtered_scan_topic'),
                'output_obstacle_topic': LaunchConfiguration('obstacle_topic'),
                'detection_angle_min_deg': ParameterValue(
                    LaunchConfiguration('detection_angle_min_deg'),
                    value_type=float,
                ),
                'detection_angle_max_deg': ParameterValue(
                    LaunchConfiguration('detection_angle_max_deg'),
                    value_type=float,
                ),
                'obstacle_distance_threshold': ParameterValue(
                    LaunchConfiguration('obstacle_distance_threshold'),
                    value_type=float,
                ),
            }],
        ),
    ])
