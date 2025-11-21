#!/bin/bash
xhost +local:docker

# --- Configuration ---
# The name of the Docker image you pulled
IMAGE_NAME="dudulrx0601/ugv_jetson_ros_humble:v1028"

# The permanent name for your local container
CONTAINER_NAME="ugv_jetson_ros_humble"

# The full path to your workspace on the HOST
HOST_WS_DIR=$(dirname "$(realpath "$0")")
# --- End Configuration ---

# Check if a container with that name already exists
if [ -z "$(docker ps -a -q -f "name=${CONTAINER_NAME}")" ]; then
    #
    # 1. CONTAINER DOES NOT EXIST: Run the one-time setup
    #
    echo "--- Container '${CONTAINER_NAME}' not found. ---"
    echo "--- Running one-time setup... ---"

docker run -it --name ${CONTAINER_NAME} -v ${HOST_WS_DIR}:/home/ws/ugv_ws -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' --network=host --privileged ${IMAGE_NAME} /bin/bash

    echo "--- One-time setup complete. ---"
    echo "--- YOU MUST NOW RE-RUN YOUR SOFTWARE SETUP ---"

else
    #
    # 2. CONTAINER EXISTS: Run the normal daily start-up
    #
    echo "--- Found container '${CONTAINER_NAME}'. ---"
    echo "--- Starting and entering... ---"

    # 1. Start the container (does nothing if already running)
    docker start ${CONTAINER_NAME} > /dev/null

    # 2. Exec into it (with all our variables)
    docker exec -it \
      -e DISPLAY=$DISPLAY \
      -e UGV_MODEL="ugv_beast" \
      ${CONTAINER_NAME} /bin/bash -c "service ssh start && /bin/bash"

fi
