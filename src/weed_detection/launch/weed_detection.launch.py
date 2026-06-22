import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('weed_detection'),
        'config',
        'homography.yaml',
    )

    camera_namespace = 'go_pro'

    go_pro_share = get_package_share_directory('camera_cpp')
    go_pro_launch_path = os.path.join(go_pro_share, 'launch', 'go_pro.launch.py')
    include_go_pro = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(go_pro_launch_path)
    )

    image_proc_share = get_package_share_directory('image_proc')
    image_proc_launch_path = os.path.join(image_proc_share, 'launch', 'image_proc.launch.py')
    include_image_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(image_proc_launch_path),
        launch_arguments={'camera_namespace': camera_namespace}.items(),
    )

    weed_detector_node = Node(
        package='weed_detection',
        executable='weed_detector',
        name='weed_detector',
        parameters=[config],
        output='screen'
    )

    weed_sequencer_node = Node(
        package='weed_detection',
        executable='weed_sequencer',
        name='weed_sequencer',
        output='screen'
    )

    return LaunchDescription([
        include_go_pro,
        include_image_proc,
        weed_sequencer_node,
        weed_detector_node,
    ])
