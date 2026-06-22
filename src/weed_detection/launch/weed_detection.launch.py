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
            package='camera_cpp',
            executable='go_pro',
            name='image_publisher',
        ),
        Node(
            package='image_proc',
            executable='rectify_node',
            name='gopro_rectifier',
            remappings=[
                ('image', '/go_pro/image'),
                ('camera_info', '/go_pro/camera_info'),
                ('image_rect', '/go_pro/image_rect'),
                ('image_rect_color', '/go_pro/image_rect_color')
            ]
        ),
        Node(
            package='weed_detection',
            executable='weed_detector',
            name='weed_detector',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='weed_detection',
            executable='weed_sequencer',
            name='weed_sequencer',
            output='screen',
        ),
    ])
