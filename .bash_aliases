# Set the ROS 2 workspace directory variable
ROS_FULL_DIR=/home/drims
ROS_WS_DIR=/home/drims/drims_ws

# Alias to build the ROS 2 workspace without tests and source the setup script
alias rbuild='(cd ${ROS_WS_DIR} && colcon build --cmake-args -DBUILD_TESTING=OFF && source ${ROS_WS_DIR}/install/setup.sh)'

# Alias to clean the ROS 2 workspace (removes install, log, and build directories)
alias rclean='(cd ${ROS_FULL_DIR} && rm -rf install/ log/ build/)'

# Alias to build with symlink install
alias rfullbuild='(cd ${ROS_FULL_DIR} && colcon build --symlink-install --continue-on-error)'

# Alias to launch with fake hardware (simulation)
alias rlaunch_fakerobot='ros2 launch drims2_description ur5e_2fg7_start.launch.py fake:=true'

# Alias to update the calibration files of the robot
alias rupdate_calib='ros2 launch ur_calibration calibration_correction.launch.py target_filename:="${HOME}/calibrations/my_robot_calibration.yaml"'

# Alias to launch with real hardware
alias rlaunch_realrobot='ros2 launch drims2_description ur5e_2fg7_start.launch.py fake:=false kinematics_params_file:="${ROS_WS_DIR}/calibrations/my_robot_calibration.yaml"'

# Alias to source the setup script
alias rsource='source ${ROS_FULL_DIR}/install/setup.bash'