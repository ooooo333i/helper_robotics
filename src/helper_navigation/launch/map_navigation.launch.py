'''
저장된 map 주행용

ros2 launch helper_navigation map_navigation.launch.py map:=/home/jiming/workspace/helper_robotics/src/helper_navigation/maps/helper_map.yaml rviz:=true motor:=true
'''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    map_file = LaunchConfiguration('map')
    front_lidar_port = LaunchConfiguration('front_lidar_port')
    front_lidar_baudrate = LaunchConfiguration('front_lidar_baudrate')
    motor_port = LaunchConfiguration('motor_port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_motor = LaunchConfiguration('motor')
    use_rviz = LaunchConfiguration('rviz')

    params_file = PathJoinSubstitution([
        FindPackageShare('helper_navigation'),
        'config',
        'helper_nav2_params.yaml',
    ])
    urdf_file = PathJoinSubstitution([
        FindPackageShare('helper_description'),
        'urdf',
        'helper_robot.urdf.xacro',
    ])
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str,
    )
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('helper_navigation'),
        'rviz',
        'behavior_nav2_demo.rviz',
    ])
    motor_launch_file = PathJoinSubstitution([
        FindPackageShare('helper_control'),
        'launch',
        'motor_driver.launch.py',
    ])
    front_lidar_launch_file = PathJoinSubstitution([
        FindPackageShare('helper_perception'),
        'launch',
        'front_lidar_slam.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to the saved map yaml file.',
        ),
        DeclareLaunchArgument(
            'front_lidar_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for the front LiDAR.',
        ),
        DeclareLaunchArgument(
            'front_lidar_baudrate',
            default_value='460800',
            description='Serial baudrate for the front LiDAR.',
        ),
        DeclareLaunchArgument(
            'motor_port',
            default_value='/dev/ttyUSB2',
            description='Serial port for the motor driver.',
        ),
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
            'rviz',
            default_value='false',
            description='Start RViz for visual inspection.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([front_lidar_launch_file]),
            launch_arguments={
                'front_serial_port': front_lidar_port,
                'front_serial_baudrate': front_lidar_baudrate,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([motor_launch_file]),
            condition=IfCondition(use_motor),
            launch_arguments={
                'serial_port': motor_port,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    nav2_bringup_dir,
                    'launch',
                    'localization_launch.py',
                ])
            ]),
            launch_arguments={
                'namespace': '',
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items(),
        ),
        SetRemap(src='cmd_vel', dst='/control/cmd_vel'),
        SetRemap(src='cmd_vel_smoothed', dst='/control/cmd_vel_smoothed'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    nav2_bringup_dir,
                    'launch',
                    'navigation_launch.py',
                ])
            ]),
            launch_arguments={
                'namespace': '',
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items(),
        ),
        Node(
            package='helper_navigation',
            executable='behavior_manager',
            name='behavior_manager_node',
            output='screen',
            parameters=[{
                'behavior_cmd_topic': '/planning/behavior_cmd',
                'behavior_state_topic': '/planning/behavior_state',
                'goal_topic': '/planning/goal_pose',
                'navigate_action': 'navigate_to_pose',
                'clear_local_costmap_service': (
                    '/local_costmap/clear_entirely_local_costmap'
                ),
                'clear_global_costmap_service': (
                    '/global_costmap/clear_entirely_global_costmap'
                ),
                'speed_limit_topic': '/speed_limit',
                'avoid_replan_delay_sec': 0.25,
                'avoid_clear_costmaps': False,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_rviz),
        ),
    ])
