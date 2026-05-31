from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    helper_navigation_share = FindPackageShare('helper_navigation')
    helper_perception_share = FindPackageShare('helper_perception')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_motor = LaunchConfiguration('motor')
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
                'rviz': use_rviz,
            }.items(),
        ),
    ])
