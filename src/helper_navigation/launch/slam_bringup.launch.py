from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    helper_navigation_share = FindPackageShare('helper_navigation')
    helper_perception_share = FindPackageShare('helper_perception')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_motor = LaunchConfiguration('motor')
    front_lidar_port = LaunchConfiguration('front_lidar_port')
    front_lidar_baudrate = LaunchConfiguration('front_lidar_baudrate')
    motor_port = LaunchConfiguration('motor_port')

    use_rviz = LaunchConfiguration('rviz')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'motor',
            default_value='true',
            description='Start motor driver and cmd_vel safety gate.',
        ),
        DeclareLaunchArgument(
            'front_lidar_port',
            default_value=EnvironmentVariable(
                'AMR_FRONT_LIDAR_PORT',
                default_value=(
                    '/dev/serial/by-path/'
                    'platform-3610000.usb-usb-0:2.3:1.0-port0'
                ),
            ),
            description='Serial port for the front LiDAR.',
        ),
        DeclareLaunchArgument(
            'front_lidar_baudrate',
            default_value='460800',
            description='Serial baudrate for the front LiDAR.',
        ),
        DeclareLaunchArgument(
            'motor_port',
            default_value=EnvironmentVariable(
                'AMR_MOTOR_DRIVER_PORT',
                default_value=(
                    '/dev/serial/by-id/'
                    'usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0'
                ),
            ),
            description='Serial port for the motor driver.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz for visual inspection.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    helper_perception_share,
                    'launch',
                    'front_lidar_slam.launch.py',
                ])
            ]),
            launch_arguments={
                'front_serial_port': front_lidar_port,
                'front_serial_baudrate': front_lidar_baudrate,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    helper_navigation_share,
                    'launch',
                    'mapping_navigation.launch.py',
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'motor': use_motor,
                'motor_port': motor_port,
                'rviz': use_rviz,
            }.items(),
        ),
    ])
