from ament_index_python.packages import get_package_share_path
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='false',
                                            description='Whether to publish the tf from the original odom to the base_footprint')                                         
     
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                                         description='Whether to launch RViz2')  

    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value='bringup',
                                         description='Choose which rviz configuration to use')  
                                             
    imu_filter_config = os.path.join(              
        get_package_share_directory('ugv_bringup'),
        'param',
        'imu_filter_param.yaml'
    )
    
    # Robot state publisher (URDF)
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_description'), 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items()
    ) 
    
    # OAK-D Pro camera launch (FrostE - vision only)
    froste_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_vision'), 'launch', 'froste_camera.launch.py')
        )
    )
    
    # UGV bringup node (reads sensors from microcontroller)
    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
    )
    
    # UGV driver node (sends commands to microcontroller)
    driver_node = Node(
        package='ugv_bringup',
        executable='ugv_driver',
    )
    
    # Base node (wheel odometry)
    base_node = Node(
        package='ugv_base_node',
        executable='base_node_ekf',
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
    )
    
    # EKF node (fuses wheel odometry + IMU directly from ugv_bringup)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("ugv_bringup"), 'param', 'ekf.yaml')],
        remappings=[('/odometry/filtered', '/odometry/filtered')]  # Keep EKF output as /odometry/filtered
    )

    return LaunchDescription([
        pub_odom_tf_arg,
        use_rviz_arg,
        rviz_config_arg,
        
        robot_state_launch,
        froste_camera_launch,  # OAK-D Pro camera
        bringup_node,
        driver_node,
        base_node,
        ekf_node
    ])
