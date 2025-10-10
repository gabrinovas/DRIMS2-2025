#!/bin/bash

# Print script start banner
echo "===================="
echo "🛡️  Starting check_script.sh"
echo "===================="
echo

# Source the ROS 2 setup script for the base installation
echo "[INFO] Sourcing ROS 2 setup..."
source /opt/ros/humble/setup.bash
# Source the static workspace setup if it exists
if [ -f /home/drims/static/drims2_ws/install/setup.bash ]; then
    echo "[INFO] Sourcing static workspace setup..."
    source /home/drims/static/drims2_ws/install/setup.bash
fi

echo
echo "--------------------"
echo "🔍 Checking environment"
echo "--------------------"

# Function to check if a command exists in the environment
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check if the static workspace directory exists
if [ -d /home/drims/static/drims2_ws/src ]; then
    echo "[OK] static drims ws exists"
else
    echo "[WARN] static drims ws does not exist"
fi

# Check if ROS 2 is installed and available in the environment
if command_exists ros2; then
    echo "[OK] ROS 2 installed"
else
    echo "[ERROR] ROS 2 not installed"
fi

# Check if the user workspace is mounted (should be mounted as a volume)
if [ -d /home/drims/drims_ws/src ]; then
    echo "[OK] user drims ws mounted"
else
    echo "[WARN] user drims ws does not exist"
fi

echo
# Print script end banner
echo "===================="
echo "✅ Finished check_script.sh"
echo "===================="

