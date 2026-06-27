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
        os.environ.get(
            'MOTOR_DRIVER_PORT',
            '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0',
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Serial port for the motor driver.',
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
                        LaunchConfiguration('serial_port'),
                        value_type=str,
                    ),
                    'cmd_vel_topic': ParameterValue(
                        LaunchConfiguration('cmd_vel_topic'),
                        value_type=str,
                    ),
                    'safety_stop_enabled': False,
                },
            ],
        ),
    ])
