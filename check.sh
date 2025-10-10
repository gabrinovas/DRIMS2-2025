#!/bin/bash

# Specify the container name and image to use
CONTAINER_NAME="drims2"
IMAGE_NAME="gabrinovas/drims2:v1.0.0"
# IMAGE_NAME="my_drims2:local"

# Pull the latest image (commented out for local builds)
echo "Pulling the latest image: $IMAGE_NAME..."
docker pull $IMAGE_NAME
# docker run $IMAGE_NAME

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

# Run the Docker container with all necessary privileges, devices, and volumes for ROS2 and hardware access,
# and execute the check_script.sh inside the container
docker run -it  --user drims -v /dev:/dev -v /dev/bus/usb:/dev/bus/usb --device=/dev/bus/usb --device-cgroup-rule='c 189:* rmw'  -v /etc/udev/rules.d:/etc/udev/rules.d --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd)/drims_ws:/home/drims/drims_ws" --volume="$(pwd)/bags:/bags"  --name drims2 -w /home/drims $IMAGE_NAME bash /home/drims/check_script.sh

