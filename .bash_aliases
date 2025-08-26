ROS_WS_DIR=/home/drims/drims_ws
alias rbuild='(cd ${ROS_WS_DIR} && colcon build --cmake-args -DBUILD_TESTING=OFF && source ${ROS_WS_DIR}/install/setup.sh)'
alias rclean='(cd ${ROS_WS_DIR} && rm -rf install/ log/ build/)'