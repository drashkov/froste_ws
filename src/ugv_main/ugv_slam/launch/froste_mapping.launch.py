from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    queue_size = LaunchConfiguration('queue_size')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_queue_size = DeclareLaunchArgument(
        'queue_size', default_value='20',
        description='Queue size'
    )
    
    declare_qos = DeclareLaunchArgument(
        'qos', default_value='2',
        description='QoS used for input sensor topics'
    )
        
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description='Launch in localization mode.'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RVIZ'
    )
                            
    # Parameters for RTAB-Map SLAM - Vision-Only Configuration
    parameters = {
            "frame_id": 'base_footprint',
            'use_sim_time': use_sim_time,
            'queue_size': queue_size,
            'qos': qos,
            
            # Topic subscriptions - Vision only
            "subscribe_rgb": True,
            "subscribe_depth": True,
            'subscribe_scan': False,  # No LiDAR
            "subscribe_odom_info": False,
            
            # Odometry
            "odom_frame_id": "odom",
            "odom_topic": "/odom",  # Use raw odometry (no EKF for now)
            "visual_odometry": False,  # Trust wheel+IMU odometry, don't compute visual odom
            
            # Synchronization
            "approx_sync": True,
            
            # Detection rate
            "Rtabmap/DetectionRate": "3.5",
            
            # Grid mapping from depth (no laser)
            "Grid/FromDepth": "true",  # Generate 2D occupancy grid from depth
            "Grid/RayTracing": "true",  # Clear empty space for navigation
            "Grid/MaxGroundHeight": "0.15",  # Ignore depth points higher than this (m)
            "Grid/MaxObstacleHeight": "2.0",  # Maximum obstacle height
            
            # Force 2D mapping (planar constraint)
            "Reg/Force3DoF": "true",  # Only x, y, yaw - prevent map tilting
            
            # Memory management
            "Mem/IncrementalMemory": "true",  # Incremental SLAM (mapping mode)
            "Mem/InitWMWithAllNodes": "false",  # Don't load all nodes on startup
     }

    # Topic remappings for OAK-D Pro
    remappings = [
        ("rgb/image", "/oak/rgb/image_rect"),
        ("rgb/camera_info", "/oak/rgb/camera_info"),
        ("depth/image", "/oak/stereo/image_raw"),
        ("odom", "/odometry/filtered"),
    ]
    
    # SLAM mode:
    rtabmap_slam_node_slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d']  # Delete previous database on startup
    )
            
    # Localization mode (for navigation):
    rtabmap_slam_node_localization = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[
            parameters,
            {'Mem/IncrementalMemory': 'False',  # Read-only mode
             'Mem/InitWMWithAllNodes': 'True'}  # Load full map
        ],
        remappings=remappings
    )
    
    # RTAB-Map visualization (optional, use RVIZ instead)
    rtabmap_viz_node = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[parameters],
        remappings=remappings,
        condition=UnlessCondition(use_rviz)
    )

    # Robot pose publisher
    robot_pose_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('robot_pose_publisher'), 'launch'),
             '/robot_pose_publisher_launch.py']
        )
    ) 
                     
    return LaunchDescription([
        declare_use_sim_time,
        declare_queue_size,
        declare_qos,
        declare_localization,
        declare_use_rviz,

        robot_pose_publisher_launch,
        rtabmap_slam_node_slam,
        rtabmap_slam_node_localization,
        rtabmap_viz_node
    ])
