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
                    'rear_lidar_c1.launch.py',
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
                'scan_filter_node_name': 'rear_scan_filter_node',
                'obstacle_detector_node_name': 'rear_obstacle_detector_node',
                'input_scan_topic': '/perception/scan/rear/raw',
                'filtered_scan_topic': '/perception/scan/rear',
                'obstacle_topic': '/perception/obstacle/rear',
            }.items(),
        ),
    ])
