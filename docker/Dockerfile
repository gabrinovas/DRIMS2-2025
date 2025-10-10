FROM ros:humble-perception

ENV DEBIAN_FRONTEND=noninteractive

# Set timezone to UTC to avoid tzdata interactive prompt during build
RUN echo 'tzdata tzdata/Areas select Etc' | debconf-set-selections \
 && echo 'tzdata tzdata/Zones/Etc select UTC' | debconf-set-selections

# Install base tools, ROS2 Control, MoveIt, camera, and other dependencies
RUN apt-get update && apt-get install -y \
    build-essential nano tmux iputils-ping gdb cmake git \
    openssl libssl-dev libmysqlclient-dev xvfb \
    python3-colcon-common-extensions \
    python3-pip \
    libboost-all-dev \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-moveit ros-humble-depthai-ros ros-humble-moveit-ros-control-interface \
    ros-humble-behaviortree-cpp* ros-humble-ros-testing freeglut3-dev ros-humble-moveit-resources-* \
    ros-humble-realsense2-camera \
    ros-humble-ur-* \
    ros-humble-omni-base* ros-humble-play-motion2 ros-humble-diagnostic-aggregator ros-humble-gazebo*\
    libprotobuf-dev protobuf-compiler ros-humble-tf* \
    ros-humble-rmw-cyclonedds-cpp \
    net-tools iproute2 \
    libnet-dev* \
    # ROS2 Control packages for grippers and controllers
    ros-humble-gripper-controllers \
    ros-humble-position-controllers \
    ros-humble-effort-controllers \
    ros-humble-joint-trajectory-controller \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-moveit-ros-occupancy-map-monitor \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-occupancy-map-monitor \
    ros-humble-moveit-ros-perception \
    # Minimal additions for OnRobot gripper support and networking
    libmodbus-dev \
    netcat-openbsd \
    tcpdump \
    ros-humble-hardware-interface \
    ros-humble-controller-interface \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    python3-pymodbus \
    netcat-openbsd \
    iputils-ping \
    # HIKRobot camera dependencies
    ros-humble-camera-aravis2 \
    ros-humble-image-transport \
    ros-humble-camera-calibration-parsers \
    ros-humble-vision-msgs \
    ros-humble-image-pipeline \
    libaravis-dev \
    libnotify-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    python3-pymodbus \
    netcat-openbsd \
    iputils-ping

# Install additional Python and C++ dependencies, and yq binary for YAML processing
RUN apt-get install -y python3-colcon-common-extensions python3-pybind11 cmake libmodbus-dev libmongoclient-dev wget\
    && wget https://github.com/gabrinovas/tools-binaries/raw/main/yq_linux_amd64 -O /usr/bin/yq \
    # Alternative yq download (commented out)
    # && wget https://github.com/mikefarah/yq/releases/download/v4.35.1/yq_linux_amd64 -O /usr/bin/yq \
    && chmod +x /usr/bin/yq

# Install essential Python packages for robotics and OnRobot support
RUN pip3 install --no-cache-dir \
    scipy \
    depthai \
    trimesh \
    transforms3d \
    ur_rtde \
    pyyaml \
    lark \
    pymodbus \ 
    pyserial \
    opencv-python \
    Pillow

# Create user and group for workspace ownership
ARG GROUP_NAME=drims2
ARG GROUP_ID=42042
ARG USER_NAME=drims
ARG USER_ID=1001

RUN groupadd -g $GROUP_ID $GROUP_NAME && \
    useradd -ms /bin/bash -u $USER_ID -g $GROUP_ID $USER_NAME && \
    echo "$USER_NAME:$USER_NAME" | chpasswd

# Create workspace directory as root
RUN mkdir -p /home/$USER_NAME/static/drims2_ws/src

WORKDIR /home/$USER_NAME/static/drims2_ws/src

# --- Groot installation section ---

# # Optionally copy local Groot installer (commented out)
# COPY Groot2-v1.6.1-linux-installer.run /home/$USER_NAME/static/drims2_ws/src/Groot2-v1.6.1-linux-installer.run

# # Optionally download Groot installer from official website (commented out)
# RUN wget https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-linux-installer.run

# Download Groot installer from GitHub (alternative link)
RUN wget https://github.com/gabrinovas/tools-binaries/raw/main/Groot2-v1.6.1-linux-installer.run

# Make Groot installer executable and install Groot to /opt/groot
RUN chmod +x Groot2-v1.6.1-linux-installer.run
RUN ./Groot2-v1.6.1-linux-installer.run install --root /opt/groot --accept-licenses --default-answer --confirm-command
RUN ln -s /opt/groot/bin/groot2 /usr/local/bin/groot

# --- Clone code and dependencies as root ---

# Clone main motion control repo
RUN git clone https://github.com/gabrinovas/drims2_motion_control.git

# Import dependencies for motion control repo using vcs
RUN vcs import < drims2_motion_control/dependencies.repos

# Clone dice simulator repo
RUN git clone https://github.com/gabrinovas/drims2_dice_simulator.git

# Clone cells repo and filter out ABB dependencies from its .repos file
RUN git clone https://github.com/gabrinovas/drims2_cells.git
RUN yq 'del(.repositories."abb_libs/abb_librws_2.0", .repositories."abb_libs/abb_omnicore_ros2", .repositories."abb_libs/abb_egm_rws_managers", .repositories."abb_libs/abb_libegm", .repositories."abb_libs/abb_ros2_msgs")' \
    drims2_cells/deps.repos > drims2_cells/deps.filtered.repos
RUN vcs import < drims2_cells/deps.filtered.repos

# Fix ownership of workspace for the created user
RUN chown -R $USER_NAME:$GROUP_NAME /home/$USER_NAME

# Switch to the created user for further operations
USER $USER_NAME
WORKDIR /home/$USER_NAME/static/drims2_ws

# Build the entire workspace with Release configuration
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Add ROS, Gazebo, and workspace setup to bashrc for environment initialization
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
RUN echo "if [ -f ~/static/drims2_ws/install/setup.bash ]; then source ~/static/drims2_ws/install/setup.bash; fi" >> ~/.bashrc

# Set CycloneDDS as the default ROS2 middleware
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# --- Add utility scripts and camera launchers to the container ---

# Copy utility scripts to the user's home directory
COPY check_script.sh /home/$USER_NAME/check_script.sh
COPY setup_robot_connection.sh /home/$USER_NAME/setup_robot_connection.sh

# Create directory for camera launchers and copy camera-related files
RUN mkdir -p /home/$USER_NAME/cameras_launchers
COPY ./cameras_launchers/hik_camera_aravis.yaml /home/$USER_NAME/cameras_launchers/hik_camera_aravis.yaml
COPY ./cameras_launchers/hik_config.yaml /home/$USER_NAME/cameras_launchers/hik_config.yaml
COPY ./cameras_launchers/multi_camera_launch.py /home/$USER_NAME/cameras_launchers/multi_camera_launch.py
COPY ./cameras_launchers/multi_d435i_launch.py /home/$USER_NAME/cameras_launchers/multi_d435i_launch.py

# Copy integration and setup scripts
COPY setup_onrobot_network.sh /home/$USER_NAME/setup_onrobot_network.sh
COPY test_gripper_integration.sh /home/$USER_NAME/test_gripper_integration.sh

# Switch to root to set permissions and ownership for scripts and launchers
USER root

# Make all copied scripts and launcher files executable
RUN chmod +x /home/$USER_NAME/check_script.sh
RUN chmod +x /home/$USER_NAME/setup_robot_connection.sh
RUN chmod +x /home/$USER_NAME/setup_onrobot_network.sh
RUN chmod +x /home/$USER_NAME/test_gripper_integration.sh
RUN chmod +x /home/$USER_NAME/cameras_launchers/hik_camera_aravis.yaml
RUN chmod +x /home/$USER_NAME/cameras_launchers/hik_config.yaml
RUN chmod +x /home/$USER_NAME/cameras_launchers/multi_camera_launch.py
RUN chmod +x /home/$USER_NAME/cameras_launchers/multi_d435i_launch.py

# Set correct ownership for all scripts and launcher files
RUN chown $USER_NAME:$GROUP_NAME /home/$USER_NAME/setup_onrobot_network.sh
RUN chown $USER_NAME:$GROUP_NAME /home/$USER_NAME/test_gripper_integration.sh
RUN chown $USER_NAME:$GROUP_NAME /home/$USER_NAME/cameras_launchers/hik_camera_aravis.yaml
RUN chown $USER_NAME:$GROUP_NAME /home/$USER_NAME/cameras_launchers/hik_config.yaml
RUN chown $USER_NAME:$GROUP_NAME /home/$USER_NAME/cameras_launchers/multi_camera_launch.py
RUN chown $USER_NAME:$GROUP_NAME /home/$USER_NAME/cameras_launchers/multi_d435i_launch.py

# --- Additional Python and ROS tools ---

# Ensure numpy version compatibility and add workspace setup to bashrc
RUN python3 -m pip install "numpy<2"
RUN echo "source /home/drims/drims_ws/install/setup.bash" >> ~/.bashrc

# Install rqt tools for ROS2 visualization
RUN apt-get update && apt-get install -y ros-humble-rqt ros-humble-rqt-tf-tree

# --- Clone UDP bridge repo and final build ---

# Clone UDP to ROS2 bridge repository

# Switch back to user for remaining operations
USER $USER_NAME
WORKDIR /home/$USER_NAME/static/drims2_ws/src
RUN git clone https://github.com/gabrinovas/udp_to_ros2_bridge.git
WORKDIR /home/$USER_NAME/static/drims2_ws

# Final build including all packages
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 

WORKDIR /home/$USER_NAME

# Default command: start an interactive bash shell
CMD ["bash"]
