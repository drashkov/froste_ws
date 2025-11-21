#!/bin/bash

# --- Configuration ---
# The name of the Docker image you pulled
IMAGE_NAME="dudulrx0601/ugv_jetson_ros_humble:v1028"

# The permanent name for your local container
CONTAINER_NAME="ugv_jetson_ros_humble"

# The full path to your workspace on the HOST
# This script assumes it's located *inside* your ugv_ws folder
HOST_WS_DIR=$(dirname "$(realpath "$0")")
# --- End Configuration ---

# Check if a container with that name already exists
if [ -z "$(docker ps -a -q -f "name=${CONTAINER_NAME}")" ]; then
    #
    # 1. CONTAINER DOES NOT EXIST: Run the one-time setup
    #
    echo "--- Container '${CONTAINER_NAME}' not found. ---"
    echo "--- Running one-time setup... ---"
    
    # This is the full 'docker run' command we figured out
    docker run -it --name ${CONTAINER_NAME} \
        -v ${HOST_WS_DIR}:/home/ws/ugv_ws \
        --network=host \
        --privileged \
        ${IMAGE_NAME} \
        /bin/bash

    echo "--- One-time setup complete. ---"
    echo "Please run your 'apt install', 'pip install', and 'build_first.sh' scripts now."
    echo "After exiting, you can run this script again for daily use."

else
    #
    # 2. CONTAINER EXISTS: Run the normal daily start-up
    #
    echo "--- Found container '${CONTAINER_NAME}'. ---"
    echo "--- Starting and entering... ---"

    # 1. Start the container (does nothing if already running)
    docker start ${CONTAINER_NAME} > /dev/null

    # 2. Exec into it (this is our fixed command)
    docker exec -it ${CONTAINER_NAME} /bin/bash -c "service ssh start && /bin/bash"

fi
