#!/bin/bash

# Test script for OnRobot gripper integration
echo "Testing OnRobot gripper integration..."

# Source the workspace
source /opt/ros/humble/setup.bash
source /home/drims/static/drims2_ws/install/setup.bash

# Test if OnRobot packages are available
echo "Checking OnRobot packages..."
ros2 pkg list | grep onrobot

# Test URDF description
echo "Testing URDF description..."
ros2 launch onrobot_description display.launch.py &

# Wait for launch to start
sleep 5

# Kill the test launch
pkill -f "ros2 launch"

echo "OnRobot gripper integration test completed"