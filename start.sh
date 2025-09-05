#!/bin/bash

# Specify the container name and local image
CONTAINER_NAME="drims2"
IMAGE_NAME="mi_drims2:local"

echo "Using local image: $IMAGE_NAME..."

# Grant X permissions
xhost +local:root

# Remove existing container if it exists
if docker ps -a --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
    echo "Container $CONTAINER_NAME exists. Removing it..."
    docker rm -f $CONTAINER_NAME
fi

# Run container
docker run -dit \
    --user drims \
    --privileged \
    -v /dev:/dev \
    -v /dev/bus/usb:/dev/bus/usb \
    --device=/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    -v /etc/udev/rules.d:/etc/udev/rules.d \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --net=host \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/drims_ws:/home/drims/drims_ws" \
    --volume="$(pwd)/bags:/home/drims/bags" \
    --name $CONTAINER_NAME \
    -w /home/drims \
    $IMAGE_NAME

# Copy .bash_aliases
docker cp $(pwd)/.bash_aliases $CONTAINER_NAME:/home/drims/.bash_aliases

# Open bash
docker exec -it $CONTAINER_NAME bash

# Revoke X permissions
xhost -local:root
echo "X permissions revoked."

echo "Container $CONTAINER_NAME is running. You can now execute commands inside the container."
echo "To stop and remove the container, use: docker rm -f $CONTAINER_NAME"

# Note: If you want to keep the container running after exiting bash,
# you can remove the 'docker rm -f $CONTAINER_NAME' line above.