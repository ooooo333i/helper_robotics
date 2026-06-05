from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('helper_perception'),
        'config',
        'depth.yaml',
    ])

    return LaunchDescription([
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
