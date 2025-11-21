cd /home/ws/ugv_ws
colcon build --packages-select costmap_converter_msgs costmap_converter emcl2 explore_lite robot_pose_publisher teb_msgs teb_local_planner ugv_base_node ugv_interface
colcon build --packages-select ugv_bringup ugv_description ugv_gazebo ugv_nav ugv_slam ugv_tools ugv_vision --symlink-install 
source install/setup.bash 

