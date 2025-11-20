from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # 1. OAK-D Driver
    camera_conf = os.path.join(
        get_package_share_directory('froste_bringup'),
        'config',
        'camera.yaml'
    )

    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('depthai_ros_driver'), 'launch'),
            '/camera.launch.py'
        ]),
        launch_arguments={'params_file': camera_conf}.items()
    )

    # 2. Visual SLAM (The Brain)
    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                name='visual_slam_node',
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                remappings=[
                    # VSLAM gets RAW images (Proven to work for tracking)
                    ('visual_slam/image_0', '/oak/left/image_raw'),
                    ('visual_slam/camera_info_0', '/oak/left/camera_info'),
                    ('visual_slam/image_1', '/oak/right/image_raw'),
                    ('visual_slam/camera_info_1', '/oak/right/camera_info'),
                    ('visual_slam/imu', '/oak/imu/data')
                ],
                parameters=[{
                    'enable_rectified_pose': True,
                    'denoise_input_images': False,
                    'rectified_images': False,   # Essential: We are feeding raw images
                    'base_frame': 'base_link',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'enable_debug_mode': False,
                    'publish_odom_to_base_tf': True,
                    'publish_map_to_odom_tf': True,
                    'invert_odom_to_base_tf': False
                }]
            )
        ],
        output='screen'
    )

    # 3. Nvblox (The Mapper)
    # Configured to listen to /oak/stereo/image_raw in its own launch file
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('froste_bringup'), 'launch'),
            '/nvblox.launch.py'
        ]),
        launch_arguments={
            'global_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link'
        }.items()
    )

    # 4. Static Transforms
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'oak-d-base-frame']
    )

    map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # NOTE: odom_tf_node is REMOVED so VSLAM can drive the robot

    return LaunchDescription([
        camera_tf_node,
        map_tf_node,
        depthai_launch,
        visual_slam_container,
        nvblox_launch
    ])