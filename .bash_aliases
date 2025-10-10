# Set the ROS 2 workspace directory variable
ROS_WS_DIR=/home/drims/drims_ws

# Alias to build the ROS 2 workspace without tests and source the setup script
alias rbuild='(cd ${ROS_WS_DIR} && colcon build --cmake-args -DBUILD_TESTING=OFF && source ${ROS_WS_DIR}/install/setup.sh)'

# Alias to clean the ROS 2 workspace (removes install, log, and build directories)
alias rclean='(cd ${ROS_WS_DIR} && rm -rf install/ log/ build/)'