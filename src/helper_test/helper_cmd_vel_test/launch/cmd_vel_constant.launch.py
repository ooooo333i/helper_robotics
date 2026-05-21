from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='helper_cmd_vel_test',
            executable='cmd_vel_test',
            name='cmd_vel_test',
            parameters=[{
                'topic': '/control/cmd_vel',
                'mode': 'constant',
                'publish_rate': 10.0,
                'linear_x': 0.1,
                'angular_z': 0.0,
            }],
        ),
    ])
