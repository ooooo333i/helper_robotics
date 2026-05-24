from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
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

    return LaunchDescription([
        Node(
            package='helper_control',
            executable='motor_driver',
            name='motor_driver_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'serial_port': ParameterValue(
                        EnvironmentVariable(
                            'AMR_MOTOR_DRIVER_PORT',
                            default_value='/dev/ttyUSB0',
                        ),
                        value_type=str,
                    ),
                },
            ],
        ),
    ])
