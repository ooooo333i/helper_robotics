from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='helper_cmd_vel_test',
            executable='obstacle_decision_test',
            name='obstacle_decision_test',
            output='screen',
            parameters=[{
                'topic': '/perception/obstacle/fused',
                'mode': 'sequence',
                'publish_rate': 10.0,
                'clear_duration': 4.0,
                'obstacle_duration': 3.0,
            }],
        ),
    ])
