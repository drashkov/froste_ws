#!/bin/bash
# Project: Sasquatch Robotics "Frost-E"
# Purpose: Persistent Development Container with X11 & Hardware Access

# --- Configuration ---
IMAGE_NAME="nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c"
CONTAINER_NAME="froste_dev_container"
HOST_WS_DIR="$HOME/froste_ws"
# --- End Configuration ---

# Safety Check: Ensure variables are set
if [ -z "$CONTAINER_NAME" ]; then
    echo "ERROR: CONTAINER_NAME is not set. Check the configuration section."
    exit 1
fi

# 1. Enable X-Forwarding (Safe Mode)
if [ -n "$DISPLAY" ]; then
    xhost +local:docker > /dev/null 2>&1
else
    echo "WARNING: \$DISPLAY is not set. RVIZ will not work."
fi

# 2. Check if container exists
if [ -z "$(docker ps -a -q -f "name=^/${CONTAINER_NAME}$")" ]; then
    # The ^/ ... $ syntax ensures we match the EXACT name, not just a substring
    
    #
    # CASE A: CONTAINER DOES NOT EXIST -> CREATE IT
    #
    echo "--- Container '${CONTAINER_NAME}' not found. ---"
    echo "--- Creating persistent container... ---"
    
    docker run -it -d \
        --network=host \
        --privileged \
        --name ${CONTAINER_NAME} \
        --device=/dev/THS1 \
        --device-cgroup-rule='c 189:* rmw' \
        -v /dev/bus/usb:/dev/bus/usb \
        -v /dev/*:/dev/* \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev/src/froste_ws \
        -v ${HOST_WS_DIR}:/workspaces/isaac_ros-dev/src/froste_ws \
        -v /etc/localtime:/etc/localtime:ro \
        ${IMAGE_NAME} \
        /bin/bash

    echo "--- Setup complete. Entering container... ---"
    echo "--- REMINDER: You must re-run 'apt-get install' commands inside! ---"
    
    # Enter the container in the correct directory
    docker exec -it -w /workspaces/isaac_ros-dev/src/froste_ws ${CONTAINER_NAME} /bin/bash

else
    #
    # CASE B: CONTAINER EXISTS -> RESTART & ENTER
    #
    echo "--- Found existing container '${CONTAINER_NAME}'. ---"
    
    # Start if stopped
    docker start ${CONTAINER_NAME} > /dev/null

    # Enter with X11 display variable passed through
    echo "--- Entering... ---"
    docker exec -it \
      -w /workspaces/isaac_ros-dev/src/froste_ws \
      -e DISPLAY=$DISPLAY \
      ${CONTAINER_NAME} /bin/bash
fi
