#!/bin/bash

# Specify the container name and image to use
CONTAINER_NAME="drims2"
IMAGE_NAME="gabrinovas/drims2:v1"
# IMAGE_NAME="my_drims2:local"

# Pull the latest image (commented out for local builds)
echo "Pulling the latest image: $IMAGE_NAME..."
docker pull $IMAGE_NAME
# docker run $IMAGE_NAME

# Grant X server permissions to root for GUI applications in the container
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

# Run the Docker container with all necessary privileges, devices, and volumes for ROS2 and hardware access
docker run -dit  --user drims --privileged -v /dev:/dev -v /dev/bus/usb:/dev/bus/usb --device=/dev/bus/usb --device-cgroup-rule='c 189:* rmw'  -v /etc/udev/rules.d:/etc/udev/rules.d --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd)/drims_ws:/home/drims/drims_ws" --volume="$(pwd)/bags:/home/drims/bags"  --name drims2 -w /home/drims $IMAGE_NAME

# Copy bash aliases into the container for convenience
docker cp $(pwd)/.bash_aliases $CONTAINER_NAME:/home/drims/.bash_aliases

# Open an interactive shell in the container and auto-execute colcon build and source the workspace
docker exec -it $CONTAINER_NAME bash # -c "colcon build --symlink-install && source install/setup.bash && bash"
