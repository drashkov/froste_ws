cd /home/ws/ugv_ws
colcon build --packages-select costmap_converter_msgs costmap_converter emcl2 explore_lite robot_pose_publisher teb_msgs teb_local_planner ugv_base_node ugv_interface
colcon build --packages-select ugv_bringup ugv_description ugv_gazebo ugv_nav ugv_slam ugv_tools ugv_vision --symlink-install 
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "eval "$(register-python-argcomplete ros2)"" >> ~/.bashrc
echo "eval "$(register-python-argcomplete colcon)"" >> ~/.bashrc
echo "source /home/ws/ugv_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc 