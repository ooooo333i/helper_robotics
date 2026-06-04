from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
        Node(
            package='helper_perception',
            executable='scan_filter_node',
            name='front_scan_filter_node',
            output='screen',
            parameters=[{
                'input_scan_topic': '/perception/scan/raw',
                'output_scan_topic': '/perception/scan/filtered',
                'angle_min_deg': ParameterValue(100.0, value_type=float),
                'angle_max_deg': ParameterValue(-100.0, value_type=float),
                'min_valid_range': ParameterValue(0.15, value_type=float),
                'max_valid_range': ParameterValue(8.0, value_type=float),
            }],
        ),
    ])
