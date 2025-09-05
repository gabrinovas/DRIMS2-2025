#!/bin/bash

# Specify the container name and local image
CONTAINER_NAME="drims2"
IMAGE_NAME="mi_drims2:local"

echo "Using local image: $IMAGE_NAME..."

# Grant X permissions (si es necesario)
xhost +local:root

# Remove existing container if it exists
if docker ps -a --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
    echo "Container $CONTAINER_NAME exists. Removing it..."
    docker rm -f $CONTAINER_NAME
fi

# Run container and execute the check script
docker run -it \
    --user drims \
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
    --volume="$(pwd)/bags:/bags" \
    --name $CONTAINER_NAME \
    -w /home/drims \
    $IMAGE_NAME \
    bash /home/drims/check_script.sh
    # bash -c "bash /home/drims/check_script.sh; exec bash"
    # bash -c "source /home/drims/drims_ws/install/setup.bash; bash /home/drims/check_script.sh; exec bash"
    # bash -c "source /home/drims/drims_ws/install/setup.bash; ros2 run mi_drims2 check_script; exec bash"
    # bash -c "source /home/drims/drims_ws/install/setup.bash; ros2 run mi_drims2 check_script; exec bash"

# Copy .bash_aliases
docker cp $(pwd)/.bash_aliases $CONTAINER_NAME:/home/drims/.bash_aliases    
docker cp $(pwd)/check_script.sh $CONTAINER_NAME:/home/drims/check_script.sh
docker exec -it $CONTAINER_NAME chmod +x /home/drims/check_script.sh
docker exec -it $CONTAINER_NAME bash -c "source /home/drims/drims_ws/install/setup.bash; bash /home/drims/check_script.sh; exec bash"
# docker exec -it $CONTAINER_NAME bash -c "source /home/drims/drims_ws/install/setup.bash; ros2 run mi_drims2 check_script; exec bash"
# docker exec -it $CONTAINER_NAME bash -c "bash /home/drims/check_script.sh; exec bash"

# Stop and remove the container after the script execution
docker rm -f $CONTAINER_NAME
echo "Container $CONTAINER_NAME has been removed."

# Revoke X permissions (si se concedieron)
xhost -local:root
echo "X permissions revoked."

echo "Check script execution completed."

# Note: If you want to keep the container running after executing the check script,
# you can remove the 'docker rm -f $CONTAINER_NAME' line above.