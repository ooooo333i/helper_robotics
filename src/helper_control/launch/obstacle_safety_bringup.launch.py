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
    helper_perception_share = FindPackageShare('helper_perception')
    helper_control_share = FindPackageShare('helper_control')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/control/cmd_vel',
            description='Twist topic the motor driver listens to.',
        ),
        DeclareLaunchArgument(
            'stop_on_unknown',
            default_value='false',
            description='Stop when obstacle state is stale/unknown.',
        ),

        # 앞 라이다 → /perception/obstacle/range
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    helper_perception_share, 'launch',
                    'front_lidar_perception.launch.py',
                ])
            ])
        ),
        # 뎁스카메라 → /perception/obstacle/depth
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    helper_perception_share, 'launch',
                    'depth_obstacle.launch.py',
                ])
            ])
        ),
        # 앞 라이다 + 뎁스 융합 → /perception/obstacle/fused
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    helper_perception_share, 'launch',
                    'obstacle_fusion.launch.py',
                ])
            ])
        ),

        # 모터 드라이버 — 하나라도 obstacle이면 정지
        Node(
            package='helper_control',
            executable='motor_driver',
            name='motor_driver_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    helper_control_share, 'config', 'motor_driver.yaml',
                ]),
                {
                    'cmd_vel_topic': ParameterValue(
                        LaunchConfiguration('cmd_vel_topic'), value_type=str,
                    ),
                    'obstacle_topic': '/perception/obstacle/fused',
                    'obstacle_topics': [
                        '/perception/obstacle/fused',
                        '/perception/obstacle/range',
                        '/perception/obstacle/depth',
                    ],
                    'safety_stop_enabled': True,
                    'stop_on_unknown': ParameterValue(
                        LaunchConfiguration('stop_on_unknown'), value_type=bool,
                    ),
                },
            ],
        ),
    ])
