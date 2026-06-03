from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'topic',
            default_value='/control/cmd_vel_test',
            description='Twist topic to publish forward test commands.',
        ),
        DeclareLaunchArgument(
            'linear_x',
            default_value='0.03',
            description='Forward velocity in m/s.',
        ),
        Node(
            package='helper_cmd_vel_test',
            executable='cmd_vel_test',
            name='cmd_vel_forward_test',
            output='screen',
            parameters=[{
                'topic': LaunchConfiguration('topic'),
                'mode': 'constant',
                'publish_rate': 10.0,
                'linear_x': ParameterValue(
                    LaunchConfiguration('linear_x'),
                    value_type=float,
                ),
                'angular_z': 0.0,
            }],
        ),
    ])
