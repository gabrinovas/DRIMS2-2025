# Stage 2: Application setup
FROM gabrinovas/drims2-base:latest AS app-setup

# Create user and group for workspace ownership
ARG GROUP_NAME=drims2
ARG GROUP_ID=42042
ARG USER_NAME=drims
ARG USER_ID=1001

RUN groupadd -g $GROUP_ID $GROUP_NAME && \
    useradd -ms /bin/bash -u $USER_ID -g $GROUP_ID $USER_NAME && \
    echo "$USER_NAME:$USER_NAME" | chpasswd && \
    usermod -a -G dialout,plugdev,video $USER_NAME

# Create workspace directory as root
RUN mkdir -p /home/$USER_NAME/static/drims2_ws/src

# Fix bags directory permissions
RUN mkdir -p /home/$USER_NAME/bags && \
    chown -R $USER_NAME:$GROUP_NAME /home/$USER_NAME/bags && \
    chmod 755 /home/$USER_NAME/bags

WORKDIR /home/$USER_NAME/static/drims2_ws/src

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

# --- Clone OnRobot ROS2 repositories ---
RUN git clone https://github.com/gabrinovas/UR_OnRobot_ROS2.git
RUN git clone https://github.com/gabrinovas/OnRobot_ROS2_Description.git
RUN git clone https://github.com/gabrinovas/OnRobot_ROS2_Driver.git

# Initialize and update the Modbus submodule
RUN cd OnRobot_ROS2_Driver && \
    echo "Initializing and updating Modbus submodule..." && \
    git submodule init && \
    git submodule update --init --recursive

# Fix the ambiguous constructor issue in FG.hpp
RUN cd OnRobot_ROS2_Driver && \
    echo "Fixing ambiguous constructor in FG.hpp..." && \
    sed -i 's/FG(const std::string \&type, const std::string \&ip, int port, int device_address = 65);/FG(const std::string \&type, const std::string \&ip, int port, int device_address);/' include/onrobot_driver/FG.hpp && \
    sed -i 's/FG(const std::string \&type, const std::string \&device, int device_address = 65);/FG(const std::string \&type, const std::string \&device, int device_address);/' include/onrobot_driver/FG.hpp

# Also fix the constructor calls in FG.cpp to be explicit
RUN cd OnRobot_ROS2_Driver && \
    echo "Fixing FG.cpp constructor calls..." && \
    sed -i 's/gripper_ = std::make_unique<FG>(onrobot_type_, ip_address_, port_, device_address_);/gripper_ = std::make_unique<FG>(onrobot_type_, ip_address_, port_, device_address_);/' src/FG.cpp && \
    sed -i 's/gripper_ = std::make_unique<FG>(onrobot_type_, device_, device_address_);/gripper_ = std::make_unique<FG>(onrobot_type_, device_, device_address_);/' src/FG.cpp

# Check if the Modbus submodule was properly initialized
RUN cd OnRobot_ROS2_Driver && \
    echo "=== Checking Modbus submodule status ===" && \
    git submodule status && \
    echo "=== Modbus directory contents ===" && \
    ls -la include/onrobot_driver/Modbus/ && \
    echo "=== Checking for CMakeLists.txt in Modbus ===" && \
    find include/onrobot_driver/Modbus -name "CMakeLists.txt" -o -name "*.cmake" | head -10

# Import additional workspace dependencies
COPY workspace.repos /home/$USER_NAME/static/drims2_ws/src/workspace.repos
RUN vcs import < workspace.repos

# Fix ownership of ENTIRE home directory for the created user
RUN chown -R $USER_NAME:$GROUP_NAME /home/$USER_NAME

# Switch to the created user for further operations
USER $USER_NAME
WORKDIR /home/$USER_NAME/static/drims2_ws

# First, build the description packages that onrobot_driver depends on
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select onrobot_description --event-handlers console_direct+

# Now build the onrobot_driver with its dependencies available
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select onrobot_driver --event-handlers console_direct+

# Now build the rest of the workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Create a test script to verify bag creation
RUN echo '#!/bin/bash\nros2 bag record -o /home/drims/bags/test_bag /rosout' > /home/$USER_NAME/test_bag_creation.sh && \
    chmod +x /home/$USER_NAME/test_bag_creation.sh

# Add ROS, Gazebo, and workspace setup to bashrc for environment initialization
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
RUN echo "if [ -f ~/static/drims2_ws/install/setup.bash ]; then source ~/static/drims2_ws/install/setup.bash; fi" >> ~/.bashrc

# Set CycloneDDS as the default ROS2 middleware
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# --- Add utility scripts to the container ---

# Copy utility scripts to the user's home directory
COPY check_script.sh /home/$USER_NAME/check_script.sh
COPY setup_robot_connection.sh /home/$USER_NAME/setup_robot_connection.sh

# Create directory for camera launchers and copy camera-related files
RUN mkdir -p /home/$USER_NAME/cameras_launchers
COPY ./cameras_launchers/multi_d435i_launch.py /home/$USER_NAME/cameras_launchers/multi_d435i_launch.py

# Copy integration and setup scripts
COPY setup_onrobot_network.sh /home/$USER_NAME/setup_onrobot_network.sh
COPY test_gripper_integration.sh /home/$USER_NAME/test_gripper_integration.sh

# Switch to root to set permissions and ownership for scripts
USER root

# Make all copied scripts executable
RUN chmod +x /home/$USER_NAME/check_script.sh
RUN chmod +x /home/$USER_NAME/setup_robot_connection.sh
RUN chmod +x /home/$USER_NAME/setup_onrobot_network.sh
RUN chmod +x /home/$USER_NAME/test_gripper_integration.sh
RUN chmod +x /home/$USER_NAME/cameras_launchers/multi_d435i_launch.py

# Set correct ownership for all scripts
RUN chown $USER_NAME:$GROUP_NAME /home/$USER_NAME/setup_onrobot_network.sh
RUN chown $USER_NAME:$GROUP_NAME /home/$USER_NAME/test_gripper_integration.sh
RUN chown $USER_NAME:$GROUP_NAME /home/$USER_NAME/cameras_launchers/multi_d435i_launch.py

# Add workspace setup to bashrc
RUN echo "source /home/drims/drims_ws/install/setup.bash" >> /home/$USER_NAME/.bashrc

# --- Clone UDP bridge repo and final build ---

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
