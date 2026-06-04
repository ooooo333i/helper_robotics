from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    helper_description_share = FindPackageShare('helper_description')
    helper_navigation_share = FindPackageShare('helper_navigation')
    helper_perception_share = FindPackageShare('helper_perception')
    slam_toolbox_share = FindPackageShare('slam_toolbox')

    front_lidar_port = LaunchConfiguration('front_lidar_port')
    front_lidar_baudrate = LaunchConfiguration('front_lidar_baudrate')
    motor_port = LaunchConfiguration('motor_port')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params_file = PathJoinSubstitution([
        helper_navigation_share,
        'config',
        'helper_slam_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'front_lidar_port',
            default_value='/dev/ttyUSB2',
            description='Serial port for the front SLAM LiDAR.',
        ),
        DeclareLaunchArgument(
            'front_lidar_baudrate',
            default_value='460800',
            description='Serial baudrate for the front SLAM LiDAR.',
        ),
        DeclareLaunchArgument(
            'motor_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for the motor driver.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/control/cmd_vel_test',
            description='Twist topic consumed by the motor driver.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation clock if true.',
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
                    helper_description_share,
                    'launch',
                    'display.launch.py',
                ])
            ]),
        ),
        Node(
            package='helper_control',
            executable='motor_driver',
            name='motor_driver_node',
            output='screen',
            parameters=[{
                'serial_port': ParameterValue(motor_port, value_type=str),
                'cmd_vel_topic': ParameterValue(cmd_vel_topic, value_type=str),
                'safety_stop_enabled': False,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    slam_toolbox_share,
                    'launch',
                    'online_async_launch.py',
                ])
            ]),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
