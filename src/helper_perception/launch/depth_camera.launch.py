from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(
            msg=(
                'Start the depth camera driver separately, then publish '
                '/camera/depth/image_rect_raw for the MVP depth pipeline.'
            )
        ),
    ])
