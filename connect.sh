#!/bin/bash

# Variables
CONTAINER_NAME="drims2"

# Grant X server permissions to root for GUI applications in the container
#xhost +si:localuser:$(whoami)  # Alternative: grant access to current user
xhost +local:root

# Check if the container is running
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Container $CONTAINER_NAME is running. Connecting to it..."
    # Open an interactive bash shell inside the running container
    docker exec -it $CONTAINER_NAME /bin/bash
else
    echo "Container $CONTAINER_NAME is not running."
fi
