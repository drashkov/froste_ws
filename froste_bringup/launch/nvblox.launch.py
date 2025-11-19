import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    # 1. Configuration File Path
    config_file = os.path.join(
        get_package_share_directory('froste_bringup'),
        'config',
        'nvblox.yaml'
    )

    # 2. Nvblox Node (Composable)
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=[config_file],
        remappings=[
            # FIX: Target the 'camera_0' internal names
            ('camera_0/depth/image', '/oak/stereo/image_raw'),
            ('camera_0/depth/camera_info', '/oak/stereo/camera_info'),
            ('pose', '/visual_slam/tracking/vo_pose'),
        ]
    )

    # 3. Container
    container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[nvblox_node],
        output='screen'
    )

    return LaunchDescription([container])
