#!/bin/bash

# Function to start the container
enter_container() {
    container_name="dudulrx0601/ugv_jetson_ros_humble:v1028"

    echo "Entering the container..."
    docker start "$container_name"
    if [ $? -eq 0 ]; then
        echo "Container started successfully."
        echo "Executing docker exec command to open a bash shell in the container..."
        docker exec -it "$container_name" /bin/bash -c "service ssh start"
        if [ $? -eq 0 ]; then
            echo "Opened bash shell in the container."
        else
            echo "Failed to open bash shell in the container."
        fi
    else
        echo "Failed to enter the container, please check the error."
    fi
}
# Call the function
enter_container
