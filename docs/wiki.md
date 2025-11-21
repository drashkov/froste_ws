Navigation menu
Waveshare Wiki
Raspberry Pi 
AI 
Displays 
IoT 
Robotics 
MCU/FPGA 
Support
IC
Product name search
UGV Beast PI ROS2 2. RViz View Product Model
UGV Beast PI ROS2
UGV Beast

I2C, UART
TTL Serial Bus Servo Control Interface
2. Rviz View product model
By default, you have completed the main program and remotely connected to the Docker container according to the content in Chapter 1 UGV Beast PI ROS2 1. Preparation.



2.1 View model joints
In the container, go to the workspace of the ROS 2 project for that product:

cd /home/ws/ugv_ws

Start the Rviz2 model interface:

ros2 launch ugv_description display.launch.py use_rviz:=true
At this time, you can only view the Rviz2 model interface of the product, but you cannot control the movement of the robot by sliding the slider on the control panel. Successfully start the terminal interface of the Rviz2 model. Do not close the interface, and then run the ROS2 robot driver node.



2.2 Run ROS2 robot driver node
We need to open a new Docker container terminal, click the "⭐" symbol in the left sidebar, double-click to open Docker's remote terminal, enter username: root, password: ws.

ROS2 newDocker.png

In the container, go to the workspace of the ROS 2 project for that product:

cd /home/ws/ugv_ws
Run the driver robot node, and there will be no information feedback after the node is finished running:

ros2 run ugv_bringup ugv_driver
Then, you can control the pan-tilt rotation by sliding the slider on the control panel in the picture below, and both the model and the product will rotate with your slide.

pt_base_link_to_pt_link1: Controls the rotation of the pan servo of the pan-tilt.
pt_link1_to_pt_link2: Controls the rotation of the tilt servo of the pan-tilt.
Center: Click this button and the pan-tilt will return to the center position.
Beast rviz.png

2.3 Control robot lights
For UGV series products, the driver board integrates 12V switch control interfaces of 2 channels (the actual maximum voltage will change with the battery voltage), which are controlled by the IO4 and IO5 pins of ESP32 through MOS tubes. Each channel has two corresponding interfaces, a total of 4 12V switch control interfaces. According to the default assembly method, IO4 controls the chassis headlight (the light next to the OKA camera), and IO5 controls the headlight (the light on the USB camera pan-tilt). You can control the switching of these two switches and adjust the voltage level by sending the corresponding commands to the sub-controller. However, due to the inherent delay in MOSFET control, there may not be a linear relationship between the PWM output from the ESP32's IO and the actual voltage output.

For products without LEDs, you can expand the 12.6V withstand LED on these two 12V switches (in general, 12V withstand is also acceptable for safety and battery protection, the product's UPS will not charge the battery above 12V). You can also expand other peripherals on the remaining switch control interfaces, such as a 12V withstand water gun gearbox, which can be directly connected to the interface controlled by IO5 to achieve automatic aiming and shooting functionality.

Every time a control node runs, we need to open a new Docker container terminal, click the ⭐ symbol in the left sidebar, double-click to open Docker's remote terminal, enter username: root, password: ws.

In the container, go to the workspace of the ROS 2 project for that product:

cd /home/ws/ugv_ws
Run the light control node, and you can see that the three LED lights on the robot are lit up:

ros2 topic pub /ugv/led_ctrl std_msgs/msg/Float32MultiArray "{data: [255, 255]}" -1
data: [0, 0]——The first 0 is the switch that controls the IO4 interface (chassis headlight); the second 0 is the switch that controls the IO5 interface (headlight).

The parameter variable range is 0-255. When the value of this variable is 0, the interface switches controlled by IO4 and IO5 are turned off; when this variable is 255, the voltage output by the interface switches controlled by IO4 and IO5 is close to the BAT voltage of the UPS (current UPS voltage of three lithium batteries connected in series).

Run the command to turn off the LED lights again, and you can see that all three LED lights are turned off:

ros2 topic pub /ugv/led_ctrl std_msgs/msg/Float32MultiArray "{data: [0, 0]}" -1

This tutorial section has shown you how to view the product model joints, run the robot drive nodes, and control the robot lights, and then follow the following tutorial to learn about other ROS 2 controls.


2. Rviz View product model
2.1 View model joints
2.2 Run ROS2 robot driver node
2.3 Control robot lights
To Top
Login / Create Account



Navigation menu
Waveshare Wiki
Raspberry Pi 
AI 
Displays 
IoT 
Robotics 
MCU/FPGA 
Support
IC
Product name search
UGV Beast Jetson Orin ROS2 3. Use Joystick or Keyboard Control
UGV Beast Jetson Orin ROS2
360px-UGV Beast PT Jetson ROS2 Kit.jpg

I2C, UART
TTL Serial Bus Servo Control Interface
3. Use Joystick or Keyboard Control
This section describes how to control movement using a joystick or keyboard keys. The product comes with an Xbox Bluetooth controller when shipped from the factory. By default, you have completed the main program and remotely connected to the Docker container according to the content in Chapter 1 UGV Beast Jetson Orin ROS2 1. Preparation.



3.1 Chassis drive
Before you can run the joystick control and keyboard control demos, you must first run the chassis drive node. In the previous section, we introduced the operation of one ROS2 robot drive node, and here we introduce the operation of another chassis drive node.

In the container, go to the workspace of the ROS 2 project for that product:

cd /home/ws/ugv_ws
Start the robot driver node:

ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=true
At this moment, the robot pan-tilt will be turned to the center position, and the camera will face forward, and you can view the RVIZ2 model interface of the product. When the robot rotates in place, you can see that the model will also rotate with the robot.

Beast rviz3.png

Note: If the UGV Beast tracked chassis is not displayed in the model interface, you need to press Ctrl+C to turn off the running robot drive node first. Next, switch the UGV model in the ROS 2 project to the UGV Beast, enter the following command to switch the model, and then start the robot drive node.

export UGV_MODEL=ugv_beast
source ~/.bashrc
3.2 Joystick control
Start the robot driver node and connect the joystick receiver to the Jetson Orin Nano.

Once plugged in, you'll need to run the joystick control node on a new Docker container terminal first. Open a new Docker container terminal, click the ⭐ symbol in the left sidebar, double-click to open Docker's remote terminal, enter username: root, password: jetson.

ROS2 newDocker.png

In the container, first run the following command to check whether the joystick is recognized, as shown in the figure:

ls /dev/input
ROS2 xboxls.png

js0 represents the joystick here.

In the container, go to the workspace of the ROS 2 project for that product:

cd /home/ws/ugv_ws
Run the joystick control node:

ros2 launch ugv_tools teleop_twist_joy.launch.py
Then turn on the switch on the back of the joystick, and you can control the movement of the robot when you see the red light on the joystick. Note: There are three function keys on the joystick: the key below R is used to lock or unlock, the left joystick -- forward or backward, the right joystick -- turn left or right.

You can close the joystick control node by pressing Ctrl+C.



3.3 Keyboard control
Close the joystick control node, and then run the joystick control node in the terminal window to run keyboard control node:

ros2 run ugv_tools keyboard_ctrl
Keep this window active (that is, make sure you are in the terminal window interface when operating the keys), and control the movement of the robot through the following keys:

keyboard key	Operation description	keyboard key	Operation description	keyboard key	Operation description
Letter U	Left forward	Letter I	Straight ahead	Letter O	Right forward
Letter J	Turn left	Letter K	Stop	Letter L	Turn right
Letter M	Left backward	Symbol ,	Straight backward	Symbol .	Right backward
You can close the keyboard control node by pressing Ctrl+C.


3. Use Joystick or Keyboard Control
3.1 Chassis drive
3.2 Joystick control
3.3 Keyboard control
To Top
Login / Create Account


Navigation menu
Waveshare Wiki
Raspberry Pi 
AI 
Displays 
IoT 
Robotics 
MCU/FPGA 
Support
IC
Product name search
UGV Beast Jetson Orin ROS2 5. 3D Mapping Based on Depth Camera
UGV Beast Jetson Orin ROS2
360px-UGV Beast PT Jetson ROS2 Kit.jpg

I2C, UART
TTL Serial Bus Servo Control Interface
5. 3D Mapping Based on Depth Camera
This tutorial will show you how to use the RTAB-Map algorithm to perform 3D mapping based on LiDAR and depth camera. We provide two visualization approaches for mapping. Before starting the mapping node, it is assumed that you have completed the main program and remotely connected to the Docker container according to the content in Chapter 1 UGV Beast Jetson Orin ROS2 1. Preparation.

5.1 Visualizing in RTAB-Map
RTAB-Map (Real-Time Appearance-Based Mapping) is an open source algorithm for Simultaneous Localization and Mapping (SLAM), which is widely used in robot navigation, autonomous vehicles, IoTs and other fields. It uses data from visual and lidar sensors to build an environment map and perform positioning. It is a SLAM method based on loop closure detection.

Start the visualization node of RTAB-Map:

ros2 launch ugv_slam rtabmap_rgbd.launch.py use_rviz:=false
In a new Docker container terminal, run either the joystick control or keyboard control node:

#Joystick control (make sure the joystick receiver is plugged into Jetson Orin Nano)
ros2 launch ugv_tools teleop_twist_joy.launch.py
#Keyboard control (keep the running keyboard control node active)
ros2 run ugv_tools keyboard_ctrl
In this way, you can control the movement of the chassis to realize the mapping of the surrounding environment. After the mapping is completed, press Ctrl+C to exit the mapping node, and the system will automatically save the map. The default saving path of the map is ~/.ros/rtabmap.db.



5.2 Visualizing in RViz
Start the visualization node of RTAB-Map:

ros2 launch ugv_slam rtabmap_rgbd.launch.py use_rviz:=true
In a new Docker container terminal, run either the joystick control or keyboard control node:

#Joystick control (make sure the joystick receiver is plugged into Jetson Orin Nano)
ros2 launch ugv_tools teleop_twist_joy.launch.py
#Keyboard control (keep the running keyboard control node active)
ros2 run ugv_tools keyboard_ctrl
In this way, you can control the movement of the chassis to realize the mapping of the surrounding environment. After the mapping is completed, press Ctrl+C to exit the mapping node, and the system will automatically save the map. The default saving path of the map is ~/.ros/rtabmap.db.

Login / Create Account


Navigation menu
Waveshare Wiki
Raspberry Pi 
AI 
Displays 
IoT 
Robotics 
MCU/FPGA 
Support
IC
Product name search
UGV Beast Jetson Orin ROS2 6. Auto Navigation
UGV Beast Jetson Orin ROS2
360px-UGV Beast PT Jetson ROS2 Kit.jpg

I2C, UART
TTL Serial Bus Servo Control Interface
6. Automatic Navigation
6.1 Start navigation
6.1.1 2D map navigation
Before starting navigation, first make sure you have built an environment map named map. If you have not followed the previous tutorial, you need to follow UGV Beast Jetson Orin ROS2 4. 2D Mapping Based on LiDAR or UGV Beast Jetson Orin ROS2 5. 3D Mapping Based on Depth Camera to create a map.

After the mapping is completed, then start the navigation, we provide a variety of autonomous navigation modes, you can choose one of the following autonomous navigation modes for robot navigation.



6.1.1.1 Navigation based on AMCL and EMCL
1. AMCL algorithm

Adaptive Monte Carlo Localization (AMCL) is a particle filter-based positioning algorithm in ROS 2 that uses 2D lidar to estimate the position and direction (i.e. posture) of the robot in a given known map. AMCL is mainly used for mobile robot navigation. It matches existing maps with laser sensors (such as lidar) to calculate the robot's position and direction in the map. The core idea is to represent the possible position of the robot through a large number of particles, and gradually update these particles to reduce the uncertainty of the robot's pose.

Advantages of AMCL:

Adaptive particle number: AMCL will dynamically adjust the number of particles based on the uncertainty of the robot's position.
Suitable for dynamic environments: While AMCL assumes a static environment, it can handle a small number of dynamic obstacles, such as pedestrians and other moving objects, to a certain extent, which makes it more flexible in practical applications.
Reliable positioning capability: AMCL's positioning effect in known maps is very reliable. Even if the robot's pose is initially uncertain, it can gradually converge to the correct pose.
AMCL assumes that the map is known and you cannot create the map yourself. It also relies on high-quality static maps that are matched to sensor data. If there is a big difference between the map and the real environment, the positioning effect will be affected. AMCL is often used for autonomous navigation of mobile robots. During the navigation process, the robot can determine its own pose through AMCL and rely on known maps for path planning and obstacle avoidance.

In the container, start navigation based on the AMCL algorithm. After successful startup, you can see the RViz screen of the previously built map:

ros2 launch ugv_nav nav.launch.py use_localization:=amcl use_rviz:=true
Then, you can determine the initial position of the robot based on 6.2 Initialize the robot's position.


2. EMCL algorithm

EMCL is an alternative Monte Carlo localization (MCL) package to AMCL. Unlike AMCL, KLD sampling and adaptive MCL are not implemented. Instead, extended resets and other features are implemented. EMCL does not rely entirely on adaptive particle filtering, but introduces methods such as extended reset to improve positioning performance. EMCL implements the extended reset strategy, a technique for improving the quality of particle sets to better handle uncertainty and drift in positioning.

Start the navigation based on the EMCL algorithm. After successful startup, you can see the RViz screen of the previously built map:

ros2 launch ugv_nav nav.launch.py use_localization:=emcl use_rviz:=true
Then, you can determine the initial position of the robot based on 6.2 Initialize the robot's position.



6.1.1.2 Pure positioning based on Cartographer
Cartographer is an open-source Google system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.

Cartographer system architecture overview: You can see that the optional inputs on the left include depth information, odometer information, IMU data, and fixed Frame attitude.

ROS2 Cartographer.png

For more tutorials, please refer to official document and project address.

Start pure positioning based on Cartographer. After successful startup, you can see the RViz screen of the previously built map:

Note: The navigation mode based on Cartographer's pure positioning can only be used after using Cartographer to build the map.

ros2 launch ugv_nav nav.launch.py use_localization:=cartographer use_rviz:=true
Then, you can determine the initial position of the robot based on 6.2 Initialize the robot's position.

6.1.1.3 Based on DWA and TEB algorithms
1. DWA algorithm Dynamic Window Approaches (DWA) is a suboptimal method based on predictive control theory, because it can safely and effectively avoid obstacles in an unknown environment, and has the characteristics of small computational effort, rapid response and strong operability. The DWA algorithm is a local path planning algorithm.

The core idea of ​​this algorithm is to determine a sampling speed space that satisfies the mobile robot's hardware constraints in the speed space (v, ω) based on the current position and speed status of the mobile robot, and then calculate the trajectories of the mobile robot within a certain period of time under these speed conditions. trajectory, and evaluate the trajectories through the evaluation function, and finally select the speed corresponding to the trajectory with the best evaluation as the movement speed of the mobile robot. This cycle continues until the mobile robot reaches the target point.

Start the navigation based on the DWA algorithm. After successful startup, you can see the RViz screen of the previously built map:

ros2 launch ugv_nav nav.launch.py use_localplan:=dwa use_rviz:=true
Then, you can determine the initial position of the robot based on 6.2 Initialize the robot's position.


2. TEB algorithm

TEB stands for Time Elastic Band Local Planner. This method performs subsequent corrections on the initial global trajectory generated by the global path planner to optimize the robot's motion trajectory and belongs to local path planning. During the trajectory optimization process, the algorithm has a variety of optimization objectives, including but not limited to: overall path length, trajectory running time, distance from obstacles, passing intermediate way points, and compliance with robot dynamics, kinematics, and geometric constraints.

Start the navigation based on the TEB algorithm. After successful startup, you can see the RViz screen of the previously built map:

ros2 launch ugv_nav nav.launch.py use_localplan:=teb use_rviz:=true
Then, you can determine the initial position of the robot based on 6.2 Initialize the robot's position.

6.1.2 3D map navigation
The map navigation mode introduced above is based on the 2D construction of LiDAR. For the 3D map built according to UGV Beast Jetson Orin ROS2 5. Mapping Based on Depth Camera, please refer to the navigation startup method in this subsection.

Enable nav positioning:

ros2 launch ugv_nav rtabmap_localization_launch.py
You need to wait for the 3D data to be loaded, wait for a period of time, and then you can start navigation as shown in the figure below.

In a new terminal, turn on navigation and choose one of the two navigation modes:

DWA algorithm
ros2 launch ugv_nav rtabmap_localization_launch.py
TEB algorithm
ros2 launch ugv_nav nav_rtabmap.launch.py use_localplan:=teb use_rviz:=true
Choose a navigation mode based on the map created above to start the navigation, then proceed with the following content.



6.2 Initialize the robot's position
By default, when navigation is started, the robot initially has no idea where it is and the map waits for you to provide it with an approximate starting location.

First, find the robot's location on the map and check the actual location of your robot. Manually set the robot's initial pose in RViz. Click the 2D Pose Estimate button and indicate the robot's location on the map. The direction of the green arrow is the direction the robot pan-tilt is facing forward.

ROS2 navigation1.png

Keep the operation of the navigation terminal, set the approximate initial pose of the robot, and ensure that the navigation ensures that the actual position of the robot is roughly corresponding to the ground. You can also control the robot through the keyboard on a new terminal to simply move and rotate it to assist in initial positioning.

ros2 run ugv_tools keyboard_ctrl


6.3 Send target pose
6.3.1 Single-point navigation
Select a target location for the robot on the map. You can use the Nav2 Goal tool to send the target location and direction to the robot. Indicate the location (target point) that the robot wants to navigate to automatically on the RViz map. The direction of the green arrow is the direction the robot pan-tilt is facing forward.

ROS2 navigation2.png

Once the target pose is set, the navigation will find the global path and begin navigating to move the robot to the target pose on the map. Now you can see the robot moving towards the actual target location.



6.3.2 Multi-point navigation
In the lower left corner of the RViz interface, there is a Nav2 RViz2 plug-in [Waypoint/Nav Through Poses Mode], which can switch the navigation mode. Click the [Waypoint/Nav Through Poses Mode] button to switch to the multi-point navigation mode.

Then use Nav2 Goal in the RViz2 toolbar to give multiple target points to move. After setting, click [Start Waypoint Following] in the lower left corner to start path planning navigation. The robot will move according to the order of the selected target points. After reaching the first target point, it will automatically go to the next target point without any further operation. The robot will stop if it reaches the last target point.



6.4 Change map name
According to the UGV Beast Jetson Orin ROS2 4. 2D Mapping Based on LiDAR tutorial, the default names of the maps we build are map, so the maps called by the above files that start navigation are also map. However, if you change the name of the map when building the map, you need to synchronously change the name of the map called in the startup file before starting navigation.

Open the nav.launch.py ​​script file in the directory /home/ws/ugv_ws/src/ugv_main/ugv_nav/launch. Change map.yaml to the name of the map you saved as shown below. Save and close after making changes.

ROS2 changemap.png

Then recompile the nav package, enter the product workspace in the Docker container terminal to compile the ugv_nav package:

cd /home/ws/ugv_ws
colcon build --packages-select ugv_nav --symlink-install
source ~/.bashrc
If you change the name of the saved map when you use Cartographer to create a map, in addition to the above files, you also need to change the bringup_launch_cartographer.launch.py script file in the /home/ws/ugv_ws/src/ugv_main/ugv_nav/launch/nav_bringup/ directory. Change map.pbstream to the name of the map you saved as shown below. Save and close after making changes, then recompile the ugv_nav package again.

ROS2 changemap2.png


6. Automatic Navigation
6.1 Start navigation
6.1.1 2D map navigation
6.1.1.1 Navigation based on AMCL and EMCL
6.1.1.2 Pure positioning based on Cartographer
6.1.1.3 Based on DWA and TEB algorithms
6.1.2 3D map navigation
6.2 Initialize the robot's position
6.3 Send target pose
6.3.1 Single-point navigation
6.3.2 Multi-point navigation
6.4 Change map name
To Top
Login / Create Account


Navigation menu
Waveshare Wiki
Raspberry Pi 
AI 
Displays 
IoT 
Robotics 
MCU/FPGA 
Support
IC
Product name search
UGV Beast Jetson Orin ROS2 7. Navigation and SLAM Mapping
UGV Beast Jetson Orin ROS2
360px-UGV Beast PT Jetson ROS2 Kit.jpg

I2C, UART
TTL Serial Bus Servo Control Interface
7. Navigation and SLAM mapping
This tutorial explains how to use Nav2 with SLAM. The following steps show you how to generate an occupancy raster map and move the robot using Nav2. By default, you have completed the main program and remotely connected to the Docker container according to the content in Chapter 1 UGV Beast Jetson Orin ROS2 1. Preparation.

In a Docker container, start the robot's RViz interface and navigation:

ros2 luanch ugv_nav slam_nav.launch.py use_rviz:=true


7.1 Manual exploration
In the RViz interface, you can manually post navigation points for exploration. You can use the Nav2 Goal tool to send the target location and direction to the robot. In the RViz interface, indicate the location (target point) that the robot wants to explore, and the direction of the green arrow is the direction that the robot pan-tilt is facing forward.

ROS2 slam.png

Similarly, you can also use the keyboard or joystick to explore remotely. In a new Docker container terminal, run either joystick control or keyboard control node. For details, please refer to UGV Beast Jetson Orin ROS2 3. Use Joystick or Keyboard Control Chapter:

#Joystick control (make sure the joystick receiver is plugged into Jetson Orin Nano)
ros2 launch ugv_tools teleop_twist_joy.launch.py
#Keyboard control (keep the running keyboard control node active)
ros2 run ugv_tools keyboard_ctrl
Alternatively, you can control the robot movement through the web for exploration, please refer to the UGV Beast Jetson Orin ROS2 9. Web-based Control Tool Chapter.

In this way, you can control the movement of the chassis to explore the surrounding environment. As you control the movement of the chassis, the map that the robot has moved will gradually be displayed in the RViz interface.

7.2 Automatic exploration
In addition to manual exploration, you can also directly let the robot explore the map automatically. Note: Automatic exploration needs to be carried out in a closed area. It is generally recommended to conduct automatic exploration in a closed small room, or in the Gazebo simulation map. For details, please refer to UGV Beast Jetson Orin ROS2 11. Gazebo Simulation Debugging.

Keep the robot and navigation instructions running on RViz interface, and then run the automatic exploration instructions in a new Docker container:

ros2 launch explore_lite explore.launch.py
The robot will move and explore within the enclosed area, and the explored area will also be displayed on the RViz interface.



7.3 Save map
After navigation and exploration, you can save the map in a new Docker container terminal. Open a new Docker container terminal, click the ⭐ symbol in the left sidebar, double-click to open Docker's remote terminal, enter username: root, password: jetson.

In the container, go to the workspace of the ROS 2 project for that product:

cd /home/ws/ugv_ws
Add executable permissions to the map saving script:

chmod +x ./save_2d_gmapping_map.sh
Then run the map saving script, as shown below, the map is saved successfully:

./save_2d_gmapping_map.sh
ROS2 savegmapping.png

The details in this script are as follows:

cd /home/ws/ugv_ws/src/ugv_main/ugv_nav/maps
ros2 run nav2_map_server map_saver_cli -f ./map
After executing the above script file, a 2D raster map named map will be saved. The map is saved in the /home/ws/ugv_ws/src/ugv_main/ugv_nav/maps directory. You can see that two files are generated in the above directory, one is map.pgm and the other is map.yaml.

map.pgm: This is a raster image of the map (usually a grayscale image file);
map.yaml: This is the configuration file of the map.

7. Navigation and SLAM mapping
7.1 Manual exploration
7.2 Automatic exploration
7.3 Save map
To Top
Login / Create Account


Navigation menu
Waveshare Wiki
Raspberry Pi 
AI 
Displays 
IoT 
Robotics 
MCU/FPGA 
Support
IC
Product name search
UGV Beast Jetson Orin ROS2 10. Command Interaction
UGV Beast Jetson Orin ROS2
360px-UGV Beast PT Jetson ROS2 Kit.jpg

I2C, UART
TTL Serial Bus Servo Control Interface
10. Command interaction
This tutorial shows you how to use commands to control the robot for basic movement and to move the robot to a navigation point. Before starting this tutorial, it is assumed that you have completed the main program and remotely connected to the Docker container according to the content in Chapter 1 UGV Beast Jetson Orin ROS2 1. Preparation.

First, start the robot behavior control interface in the container and keep this command running:

ros2 run ugv_tools behavior_ctrl


10.1 Basic control
Before performing basic control, you must first ensure that the robot is placed on the ground. The robot needs to judge whether the distance traveled by the robot has completed the goal based on the odometer.

Next open a new Docker container terminal, click the ⭐ symbol in the left sidebar, double-click to open Docker's remote terminal, enter username: root, password: jetson.

In the container, go to the workspace of the ROS 2 project for that product:

cd /home/ws/ugv_ws
Start the car driver node and keep this command running:

ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=true
At this moment, the robot pan-tilt will be turned to the center position, and the camera will face forward, and you can view the RVIZ2 model interface of the product. When you rotate the car manually, you can see that the model will also rotate along with it.



10.1.1 Move forward
Then in a new Docker container, send the action target of the robot's moving forward, and data is the distance the robot travels, in meters:

ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"drive_on_heading\", \"data\": 0.5}]'}"
At this time, the robot will advance the distance set by data in the command, and you can see that the model in the Rviz2 model interface of the product will also move forward together.

10.1.2 Move backward
Send the action target of the robot's moving backward, and data is the distance the robot travels, in meters:

ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"back_up\", \"data\": 0.5}]'}"
At this time, the robot will retreat the set distance, and you can see that the model in the Rviz2 model interface of the product will also move backward together.

10.1.3 Rotate
Send the action target of the robot rotation (the unit of data is degrees, positive number means to turn left, negative number means to turn right):

ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"spin\", \"data\": -50}]'}"
At this time, the robot will rotate to the set angle, and you can see that the model in the Rviz2 model interface of the product will also rotate along with it.

10.1.4 Stop
Send the action target of the robot to stop:

ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"stop\", \"data\": 0}]'}"
At this time, the robot will stop moving, and you can see that the model in the Rviz2 model interface of the product will also stop moving. Normally, the robot will stop moving after the value of the given data parameter is finished.



10.2 Get current point location
Before interacting with the following tutorial commands, you need to enable navigation first. Before enabling navigation, make sure you have built an environment map. If you have not followed the previous tutorial, you need to follow UGV Beast Jetson Orin ROS2 4. 2D Mapping Based on LiDAR or UGV Beast Jetson Orin ROS2 5. 3D Mapping Based on Depth Camera to create a map.

After the map construction is completed, place the robot at the actual location of the map. The robot needs to judge whether it has completed the movement to the target location based on the odometer, and run the command to start navigation in a new container:

ros2 launch ugv_nav nav.launch.py use_rviz:=true
There is no use_localization specified in this command, and the navigation based on the AMCL algorithm is used by default, as detailed in UGV Beast Jetson Orin ROS2 6. Auto Navigation section.

You can save several target positions as navigation points before using commands to control the robot to move to navigation points. First get the current point location information:

ros2 topic echo /robot_pose --once
ROS2 pose.png



10.3 Save as navigation point
Save the location information obtained previously as a navigation point, in the following command, data is the name of the navigation point, which can be set to any letter between a~g:

ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"save_map_point\", \"data\": \"a\"}]'}"
ROS2 pose1.png

You can use Nav2 Goal in the RViz navigation interface to move the robot to the next target position and save it as the next navigation point.



10.4 Move to navigation point
After saving the navigation point, send a control command to let the robot move to the corresponding navigation point, and the data is the name of the previously saved navigation point:

ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"pub_nav_point\", \"data\": \"a\"}]'}"
ROS2 pose2.png

The saved navigation point information will also be stored in the map_points.txt file, which is located in the /home/ws/ugv_ws/ directory.


10. Command interaction
10.1 Basic control
10.1.1 Move forward
10.1.2 Move backward
10.1.3 Rotate
10.1.4 Stop
10.2 Get current point location
10.3 Save as navigation point
10.4 Move to navigation point
To Top
Login / Create Account


Navigation menu
Waveshare Wiki
Raspberry Pi 
AI 
Displays 
IoT 
Robotics 
MCU/FPGA 
Support
IC
Product name search
UGV Beast Jetson Orin ROS2 11. Gazebo Simulation Debugging
UGV Beast Jetson Orin ROS2
360px-UGV Beast PT Jetson ROS2 Kit.jpg

I2C, UART
TTL Serial Bus Servo Control Interface
11. Gazebo Simulation Debugging
This chapter mainly introduces the simulation and debugging of robots. When we don’t have a physical robot at hand, we can verify robot algorithms, architecture, etc. through Gazebo, a three-dimensional robot physics simulation platform. We provide Gazebo robot models and complete function packages for simulation and debugging on virtual machines to help users conduct system verification and testing in the early stages of development.



11.1 Gazebo introduction
Gazebo is a 3D dynamic simulator that accurately and efficiently simulates groups of robot in complex indoor and outdoor environments. Although similar to game engines, Gazebo offers higher-fidelity physics simulations, supporting complex environment and sensor simulations. Users can create 3D models, set up environments, and simulate various sensors (such as lidar, cameras, etc.).

Key aspects of the Gazebo simulation platform include:

Physics engine: Gazebo uses multiple physics engines (such as ODE, Bullet, DART, etc.) to provide accurate dynamics simulation and can handle physical phenomena such as collision, friction, and gravity.
3D modeling: Users can use existing model libraries or create custom 3D models through software such as Blender. These models can be robots, obstacles, or environmental elements.
Environment settings: Gazebo allows users to design complex simulation environments, including cities, indoor scenes, etc., and can freely configure terrain and lighting.
Sensor simulation: Gazebo supports multiple sensor types, such as lidar, camera, IMU, etc., and can provide real-time data streams to facilitate algorithm testing.
ROS integration: The combination with ROS allows users to easily use Gazebo as a simulation environment for algorithm development and testing, and supports ROS themes and services.
User interface: Gazebo provides an intuitive graphical user interface. Users can monitor the simulation process and adjust parameters in real time through visual tools.


11.2 Load virtual machine image
We provide Ubuntu images configured with Gazebo simulation, robot models and complete feature packages for users to use directly. This tutorial is suitable for using a virtual machine on a Windows computer to load the Ubuntu image for Gazebo simulation debugging.

11.2.1 Download Ubuntu image with software configured
Use the Ubuntu image of the configured software, there is no need to install and configure the Gazebo simulation environment by yourself, and conduct the simulation test of the product according to the subsequent tutorials.

Download link:
Download and decompress the image, all files are image files. The disk file system of some virtual machines does not support separate files above 4G, so the configured Ubuntu image is divided into multiple files.

11.2.2 Install Oracle VM VirtualBox virtual machine
Download and install Oracle VM VirtualBox, which is a free virtual machine software that allows you to run a virtual operating system on your own computer. We run the virtual machine on a Windows system computer to install the Ubuntu operating system, and then install and configure ROS2 on the Ubuntu operating system to control the robotic arm.

It should be noted that although ROS2 has a Windows version, there is not much information about the Windows version of ROS2, so we provide a virtual machine solution by default to run ROS2.

Click Oracle VM VirtualBox official download link, the installation process is very simple, just keep clicking Next. If it is already installed, skip this step.



11.3 Load image into virtual machine software
On the left toolbar, click New.
600px-图片1.png

Set the name, set the type to Linux, and set the version to Ubuntu (64-bit), click Next.
600px-图片2.png

Set the memory size to 4096MB and the processor to 2, then click Next.
600px-图片3.png

Select Do not add a virtual hard disk, click Next to display the configuration of the new virtual computer, click Finish and a warning will pop up, then click Continue.
600px-图片4.png
600px-图片5.png

Select the virtual machine you just created and select Settings.
600px-图片6.png

Select Storage and click the + sign on the far right of the controller to add a virtual hard disk.
600px-图片7.png

Select Register, add the ws.vmdk image file decompressed earlier and click Choose in the lower right corner, and confirm to save.
图片8.png
600px-图片9.png

Then select Display, check Enable 3D Acceleration and click OK, double-click the virtual computer you just created on the left to run it.
600px-图片10.png

After successfully running the virtual machine computer, you can learn how to use Gazebo simulation to control the robot according to the following content.

11.4 Enter Docker container
In the host terminal, first allow unauthorized users to access the graphical interface and enter the command:

xhost +
Note: After each virtual machine rerun, you need to open the visualization in the Docker container, and this step must be performed.
Then execute the script that enters the Docker container:

. ros_humble.sh
Enter 1 to enter the Docker container, and the username will change to root, as shown in the following figure.

Gazebo docker.png



11.5 Load Gazebo robot model
This mirrored model defaults to a tracked type of UGV Beast, so we can directly load the Gazebo simulation environment and the robot model of UGV Beast, and start the corresponding ROS 2 nodes:

ros2 launch ugv_gazebo bringup.launch.py
The startup needs to wait for a short period of time. As shown below, the startup is successful. This command needs to keep running in subsequent steps.

Gazebo model.png



11.6 Use Joystick or Keyboard Control
11.6.1 Joystick control
Plug the joystick receiver into your computer, click on Devices above Oracle VM VirtualBox → USB → the name of the device with the word XBox, and the device name is preceded by a √ to indicate that the controller is connected to the virtual machine.

Press Ctrl+Alt+T to open a new terminal window, execute the script that goes into the Docker container, and enter 1 to enter the Docker container:

. ros_humble.sh
Run the joystick control node in the container:

ros2 launch ugv_tools teleop_twist_joy.launch.py
Then turn on the switch on the back of the joystick, and you can control the movement of the robot model when you see the red light on the joystick. Note: There are three function keys on the joystick: the key below R is used to lock or unlock, the left joystick -- forward or backward, the right joystick -- turn left or right.

You can close the joystick control node by pressing Ctrl+C.



11.6.2 Keyboard control
Close the joystick control node, and then run the joystick control node in the container terminal window to run keyboard control node:

ros2 run ugv_tools keyboard_ctrl
Keep this window active (that is, make sure you are in the terminal window interface when operating the keys), and control the movement of the robot model through the following keys:

keyboard key	Operation description	keyboard key	Operation description	keyboard key	Operation description
Letter U	Left forward	Letter I	Straight ahead	Letter O	Right forward
Letter J	Turn left	Letter K	Stop	Letter L	Turn right
Letter M	Left backward	Symbol ,	Straight backward	Symbol .	Right backward
You can close the keyboard control node by pressing Ctrl+C.



11.7 Map
11.7.1 2D Mapping
1. 2D Mapping based on Gmapping
Keep loading the Gazebo robot model running, press Ctrl+Alt+T to open a new terminal window, execute the script that goes into the Docker container, and enter 1 to enter the Docker container:

. ros_humble.sh
Run to launch the mapping node in the container:

ros2 launch ugv_gazebo gmapping.launch.py
Gazebo gmapping.png

At this time, the map displayed on the RViz interface will only show the area scanned by the lidar in the Gazebo simulation map. If there are still unscanned areas that need to be mapped, you can use the joystick or keyboard to control the movement of the robot to scan and map.

In a new terminal window, execute the script that goes into the Docker container, enter 1 to enter the Docker container, and run either the joystick control or keyboard control node:

#Joystick control (make sure the joystick receiver is plugged into a virtual machine)
ros2 launch ugv_tools teleop_twist_joy.launch.py
#Keyboard control (keep the running keyboard control node active)
ros2 run ugv_tools keyboard_ctrl
In this way, you can control the movement of the chassis to realize the mapping of the surrounding environment.

After the mapping is completed, keep the mapping node running. In a new terminal window, execute the script that enters the Docker container, enter 1 to enter the Docker container, and add executable permissions to the map saving script:

cd ugv_ws/
chmod +x ./save_2d_gmapping_map_gazebo.sh
Then run the map saving script, as shown below, the map is saved successfully:

./save_2d_gmapping_map_gazebo.sh
Gazebo savegmapping.png

The details in this script are as follows:

cd /home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps
ros2 run nav2_map_server map_saver_cli -f ./map
After executing the above script file, a 2D raster map named map will be saved. The map is saved in the /home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps directory. You can see that two files are generated in the above directory, one is map.pgm and the other is map.yaml.

map.pgm: This is a raster image of the map (usually a grayscale image file);
map.yaml: This is the configuration file of the map.
Then the Gmapping mapping node can be closed via Ctrl+C.



2. 2D Mapping based on Cartographer
Keep loading the Gazebo robot model running, press Ctrl+Alt+T to open a new terminal window, execute the script that goes into the Docker container, and enter 1 to enter the Docker container:

. ros_humble.sh
Run to launch the mapping node in the container:

ros2 launch ugv_gazebo cartographer.launch.py
Gazebo cartographer.png

At this time, the map displayed on the RViz interface will only show the area scanned by the lidar in the Gazebo simulation map. If there are still unscanned areas that need to be mapped, you can use the joystick or keyboard to control the movement of the robot to scan and map.

In a new terminal window, execute the script that goes into the Docker container, enter 1 to enter the Docker container, and run either the joystick control or keyboard control node:

#Joystick control (make sure the joystick receiver is plugged into a virtual machine)
ros2 launch ugv_tools teleop_twist_joy.launch.py
#Keyboard control (keep the running keyboard control node active)
ros2 run ugv_tools keyboard_ctrl
In this way, you can control the movement of the chassis to realize the mapping of the surrounding environment.

After the mapping is completed, keep the mapping node running. In a new terminal window, execute the script that enters the Docker container, enter 1 to enter the Docker container, and add executable permissions to the map saving script:

cd ugv_ws/
chmod +x ./save_2d_cartographer_map_gazebo.sh
Then run the map saving script, as shown below, the map is saved successfully:

./save_2d_cartographer_map_gazebo.sh
Gazebo savecartographer.png

The details in this script are as follows:

cd /home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps
ros2 run nav2_map_server map_saver_cli -f ./map && ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps/map.pbstream'}"
After executing the above script file, a 2D raster map named map will be saved. The map is saved in the /home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps directory. You can see that three files are generated in the directory,which are map.pgm, map.yaml and map.pbstram.

Then the Cartographer mapping node can be closed via Ctrl+C.



11.7.2 3D Mapping
1. Visualize in RTAB-Map
RTAB-Map (Real-Time Appearance-Based Mapping) is an open source algorithm for Simultaneous Localization and Mapping (SLAM), which is widely used in robot navigation, autonomous vehicles, IoTs and other fields. It uses data from visual and lidar sensors to build an environment map and perform positioning. It is a SLAM method based on loop closure detection.

Keep the loaded Gazebo robot model running and start the visualization node of RTAB-Map in the container:

ros2 launch ugv_gazebo rtabmap_rgbd.launch.py
In a new Docker container terminal, run either the joystick control or keyboard control node:

#Joystick control (make sure the joystick receiver is plugged into a virtual machine)
ros2 launch ugv_tools teleop_twist_joy.launch.py
#Keyboard control (keep the running keyboard control node active)
ros2 run ugv_tools keyboard_ctrl
In this way, you can control the movement of the chassis to realize the mapping of the surrounding environment. After the mapping is completed, press Ctrl+C to exit the mapping node, and the system will automatically save the map. The default saving path of the map is ~/.ros/rtabmap.db.

2. Visualize in RViz
Keep the loaded Gazebo robot model running and start the visualization node of RTAB-Map in the container:

ros2 launch ugv_gazebo rtabmap_rgbd.launch.py use_rviz:=true
In a new Docker container terminal, run either the joystick control or keyboard control node:

#Joystick control (make sure the joystick receiver is plugged into a virtual machine)
ros2 launch ugv_tools teleop_twist_joy.launch.py
#Keyboard control (keep the running keyboard control node active)
ros2 run ugv_tools keyboard_ctrl
In this way, you can control the movement of the chassis to realize the mapping of the surrounding environment. After the mapping is completed, press Ctrl+C to exit the mapping node, and the system will automatically save the map. The default saving path of the map is ~/.ros/rtabmap.db.

11.8 Navigate
11.8.1 Start navigation
1. 2D map navigation
Before you start navigating, make sure you have built a map of the environment called map, and if you haven't followed the previous tutorial, you need to build a map according to the previous tutorial.

After the mapping is completed, then start the navigation, we provide a variety of autonomous navigation modes, you can choose one of the following autonomous navigation modes for robot navigation.



AMCL algorithm
Adaptive Monte Carlo Localization (AMCL) is a particle filter-based positioning algorithm in ROS 2 that uses 2D lidar to estimate the position and direction (i.e. posture) of the robot in a given known map. AMCL is mainly used for mobile robot navigation. It matches existing maps with laser sensors (such as lidar) to calculate the robot's position and direction in the map. The core idea is to represent the possible position of the robot through a large number of particles, and gradually update these particles to reduce the uncertainty of the robot's pose.

Advantages of AMCL:

Adaptive particle number: AMCL will dynamically adjust the number of particles based on the uncertainty of the robot's position.
Suitable for dynamic environments: While AMCL assumes a static environment, it can handle a small number of dynamic obstacles, such as pedestrians and other moving objects, to a certain extent, which makes it more flexible in practical applications.
Reliable positioning capability: AMCL's positioning effect in known maps is very reliable. Even if the robot's pose is initially uncertain, it can gradually converge to the correct pose.
AMCL assumes that the map is known and you cannot create the map yourself. It also relies on high-quality static maps that are matched to sensor data. If there is a big difference between the map and the real environment, the positioning effect will be affected. AMCL is often used for autonomous navigation of mobile robots. During the navigation process, the robot can determine its own pose through AMCL and rely on known maps for path planning and obstacle avoidance.

In the container, start navigation based on the AMCL algorithm. After successful startup, you can see the RViz screen of the previously built map:

ros2 launch ugv_gazebo nav.launch.py use_localization:=amcl
Then, you can determine the initial position of the robot according to the subsequent tutorials.



EMCL algorithm
EMCL is an alternative Monte Carlo localization (MCL) package to AMCL. Unlike AMCL, KLD sampling and adaptive MCL are not implemented. Instead, extended resets and other features are implemented. EMCL does not rely entirely on adaptive particle filtering, but introduces methods such as extended reset to improve positioning performance. EMCL implements the extended reset strategy, a technique for improving the quality of particle sets to better handle uncertainty and drift in positioning.

Start the navigation based on the EMCL algorithm. After successful startup, you can see the RViz screen of the previously built map:

ros2 launch ugv_gazebo nav.launch.py use_localization:=emcl
Then, you can determine the initial position of the robot according to the subsequent tutorials.



Pure positioning based on Cartographer
Cartographer is an open-source Google system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.

Cartographer system architecture overview: You can see that the optional inputs on the left include depth information, odometer information, IMU data, and fixed Frame attitude.

ROS2 Cartographer.png

For more tutorials, please refer to official document and project address.

Start pure positioning based on Cartographer. After successful startup, you can see the RViz screen of the previously built map:

Note: The navigation mode based on Cartographer's pure positioning can only be used after using Cartographer to build the map.

ros2 launch ugv_gazebo nav.launch.py use_localization:=cartographer
Then, you can determine the initial position of the robot according to the subsequent tutorials.



DWA algorithm
Dynamic Window Approaches (DWA) is a suboptimal method based on predictive control theory, because it can safely and effectively avoid obstacles in an unknown environment, and has the characteristics of small computational effort, rapid response and strong operability. The DWA algorithm is a local path planning algorithm.

The core idea of ​​this algorithm is to determine a sampling speed space that satisfies the mobile robot's hardware constraints in the speed space (v, ω) based on the current position and speed status of the mobile robot, and then calculate the trajectories of the mobile robot within a certain period of time under these speed conditions. trajectory, and evaluate the trajectories through the evaluation function, and finally select the speed corresponding to the trajectory with the best evaluation as the movement speed of the mobile robot. This cycle continues until the mobile robot reaches the target point.

Start the navigation based on the DWA algorithm. After successful startup, you can see the RViz screen of the previously built map:

ros2 launch ugv_gazebo nav.launch.py use_localplan:=dwa
Then, you can determine the initial position of the robot according to the subsequent tutorials.



TEB algorithm
TEB stands for Time Elastic Band Local Planner. This method performs subsequent corrections on the initial global trajectory generated by the global path planner to optimize the robot's motion trajectory and belongs to local path planning. During the trajectory optimization process, the algorithm has a variety of optimization objectives, including but not limited to: overall path length, trajectory running time, distance from obstacles, passing intermediate way points, and compliance with robot dynamics, kinematics, and geometric constraints.

Start the navigation based on the TEB algorithm. After successful startup, you can see the RViz screen of the previously built map:

ros2 launch ugv_gazebo nav.launch.py use_localplan:=teb
Then, you can determine the initial position of the robot according to the subsequent tutorials.



2. 3D map navigation
The map navigation mode introduced above is based on the 2D construction of LiDAR. For the 3D map built according to previous tutorial, please refer to the navigation startup method in this subsection.

Enable nav positioning:

ros2 launch ugv_gazebo rtabmap_localization_launch.py
You need to wait for the 3D data to be loaded, wait for a period of time, and then you can start navigation as shown in the figure below.

In a new terminal, turn on navigation and choose one of the two navigation modes:

DWA algorithm
ros2 launch ugv_gazebo nav_rtabmap.launch.py use_localplan:=dwa
TEB algorithm
ros2 launch ugv_ngazebo nav_rtabmap.launch.py use_localplan:=teb
Choose a navigation mode based on the map created above to start the navigation, then proceed with the following content.



11.8.2 Initialize the robot's position
By default, when navigation is started, the robot initially has no idea where it is and the map waits for you to provide it with an approximate starting location.

First, find the robot's location on the map and check the actual location of your robot. Manually set the robot's initial pose in RViz. Click the 2D Pose Estimate button and indicate the robot's location on the map. The direction of the green arrow is the direction the robot pan-tilt is facing forward.

ROS2 navigation1.png

Keep the operation of the navigation terminal, set the approximate initial pose of the robot, and ensure that the navigation ensures that the actual position of the robot is roughly corresponding to the ground. You can also control the robot through the keyboard on a new terminal to simply move and rotate it to assist in initial positioning.

ros2 run ugv_tools keyboard_ctrl


11.8.3 Send target pose
1. Single-point navigation
Select a target location for the robot on the map. You can use the Nav2 Goal tool to send the target location and direction to the robot. Indicate the location (target point) that the robot wants to navigate to automatically on the RViz map. The direction of the green arrow is the direction the robot pan-tilt is facing forward.

ROS2 navigation2.png

Once the target pose is set, the navigation will find the global path and begin navigating to move the robot to the target pose on the map. Now you can see the robot moving towards the actual target location.



2. Multi-point navigation
In the lower left corner of the RViz interface, there is a Nav2 RViz2 plug-in [Waypoint/Nav Through Poses Mode], which can switch the navigation mode. Click the [Waypoint/Nav Through Poses Mode] button to switch to the multi-point navigation mode.

Then use Nav2 Goal in the RViz2 toolbar to give multiple target points to move. After setting, click [Start Waypoint Following] in the lower left corner to start path planning navigation. The robot will move according to the order of the selected target points. After reaching the first target point, it will automatically go to the next target point without any further operation. The robot will stop if it reaches the last target point.


11. Gazebo Simulation Debugging
11.1 Gazebo introduction
11.2 Load virtual machine image
11.2.1 Download Ubuntu image with software configured
11.2.2 Install Oracle VM VirtualBox virtual machine
11.3 Load image into virtual machine software
11.4 Enter Docker container
11.5 Load Gazebo robot model
11.6 Use Joystick or Keyboard Control
11.6.1 Joystick control
11.6.2 Keyboard control
11.7 Map
11.7.1 2D Mapping
1. 2D Mapping based on Gmapping
2. 2D Mapping based on Cartographer
11.7.2 3D Mapping
1. Visualize in RTAB-Map
2. Visualize in RViz
11.8 Navigate
11.8.1 Start navigation
1. 2D map navigation
2. 3D map navigation
11.8.2 Initialize the robot's position
11.8.3 Send target pose
1. Single-point navigation
2. Multi-point navigation
To Top
Login / Create Account

