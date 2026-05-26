from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    slam_toolbox_dir = FindPackageShare('slam_toolbox')

    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    navigate_action = PathJoinSubstitution(['/', namespace, 'navigate_to_pose'])
    clear_local_costmap_service = PathJoinSubstitution([
        '/',
        namespace,
        'local_costmap',
        'clear_entirely_local_costmap',
    ])
    clear_global_costmap_service = PathJoinSubstitution([
        '/',
        namespace,
        'global_costmap',
        'clear_entirely_global_costmap',
    ])
    speed_limit_topic = PathJoinSubstitution(['/', namespace, 'speed_limit'])

    urdf_file = PathJoinSubstitution([
        FindPackageShare('helper_description'),
        'urdf',
        'helper_robot.urdf.xacro',
    ])
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str,
    )

    default_params_file = PathJoinSubstitution([
        FindPackageShare('helper_navigation'),
        'config',
        'helper_nav2_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='planning',
            description='Namespace for Nav2 nodes.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Nav2 parameters file.',
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
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    slam_toolbox_dir,
                    'launch',
                    'online_async_launch.py',
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    nav2_bringup_dir,
                    'launch',
                    'navigation_launch.py',
                ])
            ]),
            launch_arguments={
                'namespace': namespace,
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
                'navigate_action': navigate_action,
                'clear_local_costmap_service': clear_local_costmap_service,
                'clear_global_costmap_service': clear_global_costmap_service,
                'speed_limit_topic': speed_limit_topic,
            }],
        ),
    ])
