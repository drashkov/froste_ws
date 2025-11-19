from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # 1. OAK-D Driver
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('depthai_ros_driver'), 'launch'),
            '/camera.launch.py'
        ]),
        launch_arguments={
            'pointcloud.enable': 'false',
            'imu.enable': 'true'
        }.items()
    )

    # 2. Visual SLAM (The Brain)
    # We configure it, but we don't trust it to publish TFs yet.
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
                    ('/stereo_camera/left/image', '/oak/left/image_raw'),
                    ('/stereo_camera/right/image', '/oak/right/image_raw'),
                    ('/stereo_camera/left/camera_info', '/oak/left/camera_info'),
                    ('/stereo_camera/right/camera_info', '/oak/right/camera_info'),
                    ('/visual_slam/imu', '/oak/imu/data')
                ],
                parameters=[{
                    'enable_rectified_pose': True,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'base_frame': 'base_link',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'enable_debug_mode': False,
                    'publish_odom_to_base_tf': True,
                    'publish_map_to_odom_tf': False, # DISABLE VSLAM MAP TF (We provide it manually below)
                    'invert_odom_to_base_tf': False
                }]
            )
        ],
        output='screen'
    )

    # 3. Nvblox (The Mapper)
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

    # 4. RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('isaac_ros_visual_slam'), 'rviz', 'default.cfg.rviz')]
    )

    # --- STATIC TRANSFORMS (The "Skeleton") ---
    # TF 1: Camera -> Robot (Mounting Point)
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'oak-d-base-frame']
    )

    # TF 2: Map -> Odom (Global Anchor)
    map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # TF 3: Odom -> Base_Link (THE PIN)
    # This forces the robot to exist at (0,0,0).
    # We use this because VSLAM is currently failing to publish it.
    odom_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    return LaunchDescription([
        camera_tf_node,
        map_tf_node,
        odom_tf_node,      # <--- ADD THIS
        depthai_launch,
        visual_slam_container,
        nvblox_launch,
        rviz_node
    ])
