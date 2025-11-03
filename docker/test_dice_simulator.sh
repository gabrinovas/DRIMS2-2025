#!/bin/bash
# test_dice_simulator.sh

echo "=== Testing Dice Simulator Environment ==="

# Test 1: Check Python dependencies
echo "1. Checking Python dependencies..."
python3 -c "
try:
    import trimesh
    import rtree
    import networkx
    import shapely
    import numpy
    import rclpy
    from tf_transformations import quaternion_from_euler
    print('✓ All Python dependencies OK')
except ImportError as e:
    print('✗ Missing dependency:', e)
    exit 1
"

# Test 2: Check ROS 2 workspace
echo "2. Checking ROS 2 workspace..."
source /opt/ros/humble/setup.bash
if [ -f /home/drims/static/drims2_ws/install/setup.bash ]; then
    source /home/drims/static/drims2_ws/install/setup.bash
    echo "✓ ROS 2 workspace sourced successfully"
else
    echo "✗ ROS 2 workspace not found"
    exit 1
fi

# Test 3: Check dice simulator package
echo "3. Checking dice simulator package..."
if [ -d "/home/drims/static/drims2_ws/src/drims2_dice_simulator" ]; then
    echo "✓ Dice simulator package found"
    
    # Check if package builds
    cd /home/drims/static/drims2_ws
    if colcon build --packages-select drims2_dice_simulator --cmake-args -DCMAKE_BUILD_TYPE=Release; then
        echo "✓ Dice simulator package builds successfully"
    else
        echo "✗ Dice simulator package failed to build"
        exit 1
    fi
else
    echo "✗ Dice simulator package not found"
    exit 1
fi

# Test 4: Check mesh files
echo "4. Checking mesh files..."
MESH_PATH="/home/drims/static/drims2_ws/install/drims2_dice_simulator/share/drims2_dice_simulator/urdf/simplify_Die-OBJ.obj"
if [ -f "$MESH_PATH" ]; then
    echo "✓ Dice mesh file found: $MESH_PATH"
else
    echo "✗ Dice mesh file not found at: $MESH_PATH"
    # Check source location
    SRC_MESH_PATH="/home/drims/static/drims2_ws/src/drims2_dice_simulator/urdf/simplify_Die-OBJ.obj"
    if [ -f "$SRC_MESH_PATH" ]; then
        echo "  Found at source location: $SRC_MESH_PATH"
    else
        echo "  Not found in source location either"
        exit 1
    fi
fi

# Test 5: Check if node can be run
echo "5. Testing node execution..."
timeout 5s ros2 run drims2_dice_simulator dice_spawner --help > /dev/null 2>&1
if [ $? -eq 0 ] || [ $? -eq 124 ]; then
    echo "✓ Dice spawner node can be executed"
else
    echo "✗ Dice spawner node failed to execute"
    exit 1
fi

echo ""
echo "=== All tests passed! ==="
echo "The dice simulator environment is ready."
echo ""
echo "To use the dice simulator:"
echo "1. Start your robot and MoveIt2: ros2 launch your_robot_moveit_config your_robot.launch.py"
echo "2. Spawn the dice: ros2 launch drims2_dice_simulator spawn_dice.launch.py"
echo "3. Test dice identification: ros2 service call /dice_identification drims2_msgs/srv/DiceIdentification"   