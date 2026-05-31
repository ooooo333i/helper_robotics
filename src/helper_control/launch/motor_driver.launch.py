import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('helper_control'),
        'config',
        'motor_driver.yaml',
    ])
    serial_port = os.environ.get(
        'AMR_MOTOR_DRIVER_PORT',
        os.environ.get('MOTOR_DRIVER_PORT', '/dev/ttyUSB1'),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'safety_stop_enabled',
            default_value='true',
            description='Enable obstacle safety stop gate.',
        ),
        DeclareLaunchArgument(
            'stop_on_unknown',
            default_value='false',
            description='Stop when obstacle decision is unknown or stale.',
        ),
        DeclareLaunchArgument(
            'obstacle_topic',
            default_value='/perception/obstacle/fused',
            description='ObstacleDecision topic consumed by the motor driver.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/control/cmd_vel_safe',
            description='Twist topic consumed by the motor driver.',
        ),
        Node(
            package='helper_control',
            executable='cmd_vel_safety_gate',
            name='cmd_vel_safety_gate_node',
            output='screen',
            parameters=[{
                'input_cmd_vel_topic': '/control/cmd_vel_smoothed',
                'output_cmd_vel_topic': '/control/cmd_vel_safe',
                'behavior_topic': '/planning/behavior_state',
            }],
        ),
        Node(
            package='helper_control',
            executable='motor_driver',
            name='motor_driver_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'serial_port': ParameterValue(
                        serial_port,
                        value_type=str,
                    ),
                    'cmd_vel_topic': ParameterValue(
                        LaunchConfiguration('cmd_vel_topic'),
                        value_type=str,
                    ),
                    'obstacle_topic': ParameterValue(
                        LaunchConfiguration('obstacle_topic'),
                        value_type=str,
                    ),
                    'obstacle_topics': [
                        '/perception/obstacle/fused',
                        '/perception/obstacle/range',
                        '/perception/obstacle/depth',
                    ],
                    'safety_stop_enabled': ParameterValue(
                        LaunchConfiguration('safety_stop_enabled'),
                        value_type=bool,
                    ),
                    'stop_on_unknown': ParameterValue(
                        LaunchConfiguration('stop_on_unknown'),
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
