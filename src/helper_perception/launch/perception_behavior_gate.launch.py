from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('helper_perception'),
        'config',
        'behavior_gate.yaml',
    ])

    return LaunchDescription([
        Node(
            package='helper_perception',
            executable='perception_behavior_gate_node',
            name='perception_behavior_gate_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
