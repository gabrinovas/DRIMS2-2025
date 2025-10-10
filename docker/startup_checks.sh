#!/bin/bash

# Print header for startup checks
echo "=== DRIMS2 Startup Network Checks ==="

# Source ROS2 setup to ensure ROS2 commands and environment are available
source /opt/ros/humble/setup.bash
if [ -f /home/drims/static/drims2_ws/install/setup.bash ]; then
    source /home/drims/static/drims2_ws/install/setup.bash
fi

# Run robot network configuration script (sets up CycloneDDS config and checks robot connectivity)
echo ""
echo "1. Configuring robot network..."
/home/drims/setup_robot_connection.sh

# Run OnRobot Compute Box network check and environment setup
echo ""
echo "2. Checking OnRobot Compute Box..."
/home/drims/setup_onrobot_network.sh

# Run basic ROS2 environment check script (checks workspaces, ROS2 install, etc.)
echo ""
echo "3. Basic ROS2 environment check..."
/home/drims/check_script.sh

# Print completion message
echo ""
echo "=== Network checks complete ==="
echo "Environment ready for DRIMS2 operation."
echo ""

# Start main application or keep container running (exec passes control to CMD or entrypoint)
echo "Container ready for use."
exec "$@"