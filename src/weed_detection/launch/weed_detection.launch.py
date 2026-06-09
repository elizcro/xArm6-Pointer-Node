import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('weed_detection'),
        'config',
        'homography.yaml',
    )

    return LaunchDescription([
        Node(
            package='weed_detection',
            executable='weed_detector',
            name='weed_detector',
            parameters=[config],
            output='screen',
        ),
    ])
