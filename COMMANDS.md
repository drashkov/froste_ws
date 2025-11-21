View in RVIZ, do nothing

ros2 launch ugv_description display.launch.py use_rviz:=true


Bringup reads data from IMU, and raw odometry, driver controls velocity, pantil and leds

ros2 run ugv_bringup ugv_driver


