from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
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

    params_file = PathJoinSubstitution([
        FindPackageShare('helper_navigation'),
        'config',
        'helper_nav2_fake_params.yaml',
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
    use_rviz = LaunchConfiguration('rviz')
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('helper_navigation'),
        'rviz',
        'behavior_nav2_demo.rviz',
    ])

    return LaunchDescription([
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
                'use_sim_time': False,
            }],
        ),
        Node(
            package='helper_navigation',
            executable='demo_cmd_vel_odom',
            name='demo_cmd_vel_odom_node',
            output='screen',
        ),
        Node(
            package='helper_control',
            executable='fake_scan',
            name='fake_scan',
            output='screen',
        ),
        GroupAction([
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
                    'use_sim_time': 'False',
                    'params_file': params_file,
                }.items(),
            ),
            Node(
                package='helper_navigation',
                executable='behavior_manager',
                name='behavior_manager_node',
                output='screen',
                parameters=[{
                    'navigate_action': 'navigate_to_pose',
                    'clear_local_costmap_service': (
                        '/local_costmap/clear_entirely_local_costmap'
                    ),
                    'clear_global_costmap_service': (
                        '/global_costmap/clear_entirely_global_costmap'
                    ),
                    'speed_limit_topic': '/speed_limit',
                }],
            ),
        ]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_rviz),
        ),
    ])
