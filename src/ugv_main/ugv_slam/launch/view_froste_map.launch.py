from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    database_path = LaunchConfiguration('database_path')
    
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
    
    declare_database_path = DeclareLaunchArgument(
        'database_path',
        default_value='/root/.ros/rtabmap.db',
        description='Path to RTAB-Map database file'
    )
                            
    # Parameters for RTAB-Map - Localization Mode (View Only)
    parameters = {
            "frame_id": 'base_footprint',
            'use_sim_time': use_sim_time,
            'queue_size': queue_size,
            'qos': qos,
            
            # Topic subscriptions - Vision only
            "subscribe_rgb": True,
            "subscribe_depth": True,
            'subscribe_scan': False,
            "subscribe_odom_info": False,
            
            # Odometry
            "odom_frame_id": "odom",
            "odom_topic": "/odom",
            "visual_odometry": False,
            
            # Synchronization
            "approx_sync": True,
            
            # Detection rate
            "Rtabmap/DetectionRate": "1.0",  # Slower for viewing
            
            # Localization mode (read-only, load saved map)
            "Mem/IncrementalMemory": "False",  # Read-only mode
            "Mem/InitWMWithAllNodes": "True",  # Load all nodes from database
            
            # Database
            "database_path": database_path,
     }

    # Topic remappings for OAK-D Pro
    remappings = [
        ("rgb/image", "/oak/rgb/image_rect"),
        ("rgb/camera_info", "/oak/rgb/camera_info"),
        ("depth/image", "/oak/stereo/image_raw"),
        ("odom", "/odom"),
    ]
    
    # RTAB-Map in localization mode (view saved map)
    rtabmap_localization_node = Node(
        package='rtabmap_slam', 
        executable='rtabmap', 
        output='screen',
        parameters=[parameters],
        remappings=remappings
        # No arguments - load existing database, don't delete it!
    )
    
    # RTAB-Map visualization
    rtabmap_viz_node = Node(
        package='rtabmap_viz', 
        executable='rtabmap_viz', 
        output='screen',
        parameters=[parameters],
        remappings=remappings
    )

    # Robot pose publisher
    robot_pose_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('robot_pose_publisher'), 'launch'),
             '/robot_pose_publisher_launch.py']
        )
    ) 
    
    # Launch RVIZ with a config file (if exists)
    rviz_config = os.path.join(
        get_package_share_directory('ugv_slam'),
        'rviz',
        'rtabmap_view.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen'
    )
                     
    return LaunchDescription([
        declare_use_sim_time,
        declare_queue_size,
        declare_qos,
        declare_database_path,

        robot_pose_publisher_launch,
        rtabmap_localization_node,
        rviz_node
    ])
