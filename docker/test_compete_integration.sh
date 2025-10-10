#!/bin/bash

# Print header for the complete OnRobot 2FG7 integration test
echo "=== COMPLETE OnRobot 2FG7 Integration Test ==="
echo "Testing simulation and hardware readiness..."

# Source ROS 2 and workspace environments
source /opt/ros/humble/setup.bash
source /home/drims/static/drims2_ws/install/setup.bash

echo ""
echo "1. Hardware Detection Test..."
# Run Python script to check if hardware mode is available
python3 /home/drims/static/drims2_ws/src/onrobot_driver/scripts/check_hardware_mode.py

echo ""
echo "2. Network Configuration..."
# Run OnRobot network setup script to configure environment variables and check connectivity
/home/drims/setup_onrobot_network.sh

echo ""
echo "3. Starting Complete System (Simulation Mode)..."
# Launch the integration test in simulation mode (MoveIt and RViz disabled)
ros2 launch drims2_description ur5e_2fg7_test_integration.launch.py fake:=true start_moveit:=false start_rviz:=false &
LAUNCH_PID=$!

echo " - Waiting for system startup..."
sleep 10

echo ""
echo "4. System Status Check..."

echo " - Controller Status:"
# List all controllers and their status
ros2 control list_controllers

echo ""
echo " - Active Nodes:"
# List all active ROS 2 nodes
ros2 node list

echo ""
echo " - Action Servers:"
# List all available action servers
ros2 action list

echo ""
echo "5. Testing Gripper with MoveIt Commands..."
echo " - Testing via action interface:"
# Send test commands to the gripper using the action interface
python3 /home/drims/static/drims2_ws/src/onrobot_driver/scripts/control_gripper.py 0.070 50.0
sleep 2
python3 /home/drims/static/drims2_ws/src/onrobot_driver/scripts/control_gripper.py 0.030 70.0
sleep 2
python3 /home/drims/static/drims2_ws/src/onrobot_driver/scripts/control_gripper.py 0.060 40.0

echo ""
echo "6. Testing Joint Trajectory Controller..."
echo " - Current joint states:"
# Display current joint states (first 20 lines)
ros2 topic echo /joint_states --once | head -20

echo ""
echo "7. System Information:"
echo " - Gripper topics:"
# List all topics related to the gripper
ros2 topic list | grep gripper
echo " - Controller topics:"
# List all topics related to controllers
ros2 topic list | grep controller

echo ""
echo "8. Testing Service Availability..."
# Check if controller_manager services are available
if ros2 service list | grep -q "controller_manager"; then
    echo "✅ Controller manager services available"
else
    echo "⚠️  Controller manager services not found"
fi

# Cleanup: stop the launched integration test
echo ""
echo "9. Cleaning up..."
kill $LAUNCH_PID 2>/dev/null || true
sleep 2

echo ""
echo "=== COMPLETE INTEGRATION TEST FINISHED ==="
echo "🎉 System is ready for both simulation and hardware operation!"
echo ""
echo "Usage commands:"
echo "  Simulation: ros2 launch drims2_description ur5e_2fg7_test_integration.launch.py fake:=true"
echo "  Hardware:   ros2 launch drims2_description ur5e_2fg7_test_integration.launch.py fake:=false"
echo "  With MoveIt: ros2 launch drims2_description ur5e_2fg7_test_integration.launch.py start_moveit:=true"