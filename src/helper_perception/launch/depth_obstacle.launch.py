from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('helper_perception'),
        'config',
        'depth.yaml',
    ])

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
        }.items(),
    )

    return LaunchDescription([
        realsense_launch,
        Node(
            package='helper_perception',
            executable='depth_obstacle_detector_node',
            name='depth_obstacle_detector_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='helper_perception',
            executable='depth_obstacle_cloud_node',
            name='depth_obstacle_cloud_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
