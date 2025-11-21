
#Setup

Had to sudo vim install/bash.setup so that I can set my rover model

# Running ROS


To start with xforwarding:
bash start_xforward.sh

Go into docker
docker exec -it -e DISPLAY=$DISPLAY ugv_jetson_ros_humble /bin/bash


To start rviz
ros2 launch ugv_description display.launch.py use_rviz:=true


that stupid jetson if statement was all wrong
