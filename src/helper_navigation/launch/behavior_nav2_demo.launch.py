'''
실제 로봇 없이 RViz에서 behavior -> Nav2 -> cmd_vel 흐름 확인용

RViz에서 goal 찍기
-> behavior_manager가 goal을 Nav2로 전달
-> Nav2가 속도 생성
-> safety gate가 stop 여부 확인
-> fake odom이 로봇 움직임처럼 TF/odom 생성 
-> RViz에서 움직이는 것 확인
'''

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
            default_value='true',
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

        # 실제 모터 연동했을 때는 /control/odom 받아오게 수정하면 돼
        Node(
            package='helper_navigation',
            executable='demo_cmd_vel_odom',
            name='demo_cmd_vel_odom_node',
            output='screen',
        ),
        # /perception/scan/filtered 같은 실제 scan 값 publish
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
