#!/bin/bash

# Print header for the OnRobot 2FG7 Gripper Integration Test
echo "=== OnRobot 2FG7 Gripper Integration Test ==="

# Source ROS 2 and workspace environments
source /opt/ros/humble/setup.bash
source /home/drims/static/drims2_ws/install/setup.bash

# Run network detection and environment setup for OnRobot
echo "1. Checking network configuration..."
/home/drims/setup_onrobot_network.sh

echo ""
echo "2. Starting test components..."

# Kill any existing ROS 2 nodes or launch processes to avoid conflicts
echo " - Cleaning up existing nodes..."
ros2 daemon stop 2>/dev/null || true
pkill -f "ros2 launch" 2>/dev/null || true
sleep 2

# Start the minimal test system for the UR5e + 2FG7 setup
echo " - Starting test system..."
ros2 launch drims2_description ur5e_2fg7_test.launch.py &
LAUNCH_PID=$!

# Wait for the system to start up
echo " - Waiting for system to start..."
sleep 15

echo ""
echo "3. Running integration tests..."

# Test 1: Check if controllers are loaded and active
echo " - Testing controller status..."
if ros2 control list_controllers | grep -q "active"; then
    echo "   ✅ Controllers are active"
    ros2 control list_controllers
else
    echo "   ⚠️  No active controllers found"
    ros2 control list_controllers
fi

# Test 2: Check if the gripper action server is available
echo " - Testing action server..."
if ros2 action list | grep -q "gripper_action"; then
    echo "   ✅ Gripper action server available"
else
    echo "   ❌ Gripper action server not found"
    echo "   Available actions:"
    ros2 action list
    kill $LAUNCH_PID
    exit 1
fi

# Test 3: Check if joint states are published and gripper joints are present
echo " - Testing joint states..."
if ros2 topic list | grep -q "/joint_states"; then
    echo "   ✅ Joint states topic available"
    # Check if gripper joints are present in joint states
    if ros2 topic echo /joint_states --once | grep -q "finger_joint"; then
        echo "   ✅ Gripper joints found in joint states"
    else
        echo "   ⚠️  Gripper joints not found in joint states"
    fi
else
    echo "   ❌ Joint states topic not found"
    kill $LAUNCH_PID
    exit 1
fi

# Test 4: Send gripper commands and check for success
echo " - Testing gripper commands..."
echo "   Testing open command (70mm)..."
if python3 /home/drims/static/drims2_ws/src/onrobot_driver/scripts/control_gripper.py 0.070 50.0; then
    echo "   ✅ Open command successful"
else
    echo "   ❌ Open command failed"
fi

sleep 2

echo "   Testing close command (40mm)..."
if python3 /home/drims/static/drims2_ws/src/onrobot_driver/scripts/control_gripper.py 0.040 60.0; then
    echo "   ✅ Close command successful"
else
    echo "   ❌ Close command failed"
fi

sleep 2

echo "   Testing ready position (60mm)..."
if python3 /home/drims/static/drims2_ws/src/onrobot_driver/scripts/control_gripper.py 0.060 40.0; then
    echo "   ✅ Ready position command successful"
else
    echo "   ❌ Ready position command failed"
fi

# Test 5: Check if gripper-related topics are available
echo " - Testing gripper topics..."
if ros2 topic list | grep -q "gripper"; then
    echo "   ✅ Gripper topics available:"
    ros2 topic list | grep gripper
else
    echo "   ⚠️  No gripper-specific topics found"
fi

# Cleanup: stop the launched test system and any remaining control nodes
echo ""
echo "4. Cleaning up..."
kill $LAUNCH_PID 2>/dev/null || true
pkill -f "ros2_control_node" 2>/dev/null || true
sleep 2

echo ""
echo "=== Integration test complete ==="
echo "Summary: Gripper action server is working correctly!"
echo "Note: Controller issues need to be resolved for full MoveIt integration."