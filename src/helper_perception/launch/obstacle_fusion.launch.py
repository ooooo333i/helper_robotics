from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('helper_perception'),
        'config',
        'fusion.yaml',
    ])

    return LaunchDescription([
        Node(
            package='helper_perception',
            executable='obstacle_fusion_node',
            name='obstacle_fusion_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
