from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='helper_control',
            executable='motor_driver',
            name='motor_driver_node',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/control/cmd_vel',
                'obstacle_topic': '/perception/obstacle/fused',
                'dry_run': True,
                'safety_stop_enabled': True,
                'stop_on_unknown': True,
                'cmd_timeout': 0.5,
                'obstacle_timeout': 1.0,
                'dry_run_log_period': 0.5,
            }],
        ),
        Node(
            package='helper_cmd_vel_test',
            executable='cmd_vel_test',
            name='cmd_vel_test',
            output='screen',
            parameters=[{
                'topic': '/control/cmd_vel',
                'mode': 'constant',
                'publish_rate': 10.0,
                'linear_x': 0.1,
                'angular_z': 0.0,
            }],
        ),
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
