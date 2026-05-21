from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(
            msg=(
                'Start the depth camera driver separately, then publish '
                '/perception/depth/image_raw for the MVP depth pipeline.'
            )
        ),
    ])
