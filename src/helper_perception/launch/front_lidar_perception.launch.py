from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    helper_perception_share = FindPackageShare('helper_perception')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    helper_perception_share,
                    'launch',
                    'front_lidar_c1.launch.py',
                ])
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    helper_perception_share,
                    'launch',
                    'perception_lidar.launch.py',
                ])
            ]),
            launch_arguments={
                'scan_filter_node_name': 'front_scan_filter_node',
                'obstacle_detector_node_name': 'lidar_obstacle_detector_node',
                'input_scan_topic': '/perception/scan/raw',
                'filtered_scan_topic': '/perception/scan/filtered',
                'obstacle_topic': '/perception/obstacle/range',
                'angle_min_deg': '90.0',
                'angle_max_deg': '-90.0',
                'detection_angle_min_deg': '150.0',
                'detection_angle_max_deg': '-150.0',
            }.items(),
        ),
    ])
