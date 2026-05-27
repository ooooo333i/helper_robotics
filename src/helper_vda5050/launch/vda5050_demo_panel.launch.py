from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('broker_host', default_value='localhost'),
        DeclareLaunchArgument('broker_port', default_value='1883'),
        DeclareLaunchArgument('manufacturer', default_value='helper'),
        DeclareLaunchArgument('serial_number', default_value='helper_001'),
        DeclareLaunchArgument('http_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('http_port', default_value='8088'),
        Node(
            package='helper_vda5050',
            executable='vda5050_demo_panel',
            name='vda5050_demo_panel_node',
            output='screen',
            parameters=[{
                'broker_host': LaunchConfiguration('broker_host'),
                'broker_port': LaunchConfiguration('broker_port'),
                'manufacturer': LaunchConfiguration('manufacturer'),
                'serial_number': LaunchConfiguration('serial_number'),
                'http_host': LaunchConfiguration('http_host'),
                'http_port': LaunchConfiguration('http_port'),
            }],
        ),
    ])
