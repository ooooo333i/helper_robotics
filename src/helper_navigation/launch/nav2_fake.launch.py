from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    params_file = PathJoinSubstitution([
        FindPackageShare('helper_navigation'),
        'config',
        'helper_nav2_params.yaml'
    ])

    return LaunchDescription([
        GroupAction([
            SetRemap(src='cmd_vel', dst='/control/cmd_vel'),
            SetRemap(src='cmd_vel_smoothed', dst='/control/cmd_vel_smoothed'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        nav2_bringup_dir,
                        'launch',
                        'navigation_launch.py'
                    ])
                ]),
                launch_arguments={
                    'namespace': 'planning',
                    'use_sim_time': 'False',
                    'params_file': params_file,
                }.items()
            )
        ])
    ])
