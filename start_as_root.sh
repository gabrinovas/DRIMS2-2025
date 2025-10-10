#!/bin/bash

# Specify the container name and image to use
CONTAINER_NAME="drims2"
IMAGE_NAME="smentasti/drims2:2025"

# Pull the latest image from the registry
echo "Pulling the latest image: $IMAGE_NAME..."
docker pull $IMAGE_NAME

# Allow root user in the container to access the X server for GUI applications
xhost +local:root

# Check if the container already exists
if docker ps -a | grep -q $CONTAINER_NAME; then
    echo "Container $CONTAINER_NAME exists."

    # If the container is running, stop and remove it
    if [ "$(docker inspect -f {{.State.Running}} $CONTAINER_NAME)" == "true" ]; then
        echo "Container $CONTAINER_NAME is running. Stopping it now..."
        docker stop $CONTAINER_NAME
        docker rm $CONTAINER_NAME
    else
        echo "Container $CONTAINER_NAME is not running."
        docker rm $CONTAINER_NAME
    fi
else
    echo "Container $CONTAINER_NAME does not exist."
fi

# Run the Docker container as root with all necessary privileges, devices, and volumes for ROS2 and hardware access
docker run -it --privileged -v /dev:/dev --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd)/drims_ws:/home/drims/drims_ws" --volume="$(pwd)/bags:/home/drims/bags" --name drims2 -w /home/drims $IMAGE_NAME

