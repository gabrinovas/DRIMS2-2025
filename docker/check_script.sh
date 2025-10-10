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
echo "--------------------"
echo "📷 Checking camera connectivity"
echo "--------------------"

# Check Hikrobot camera network connectivity
echo "[INFO] Testing Hikrobot camera network connectivity..."
if ping -c 2 -W 3 172.168.100.10 >/dev/null 2>&1; then
    echo "[OK] Hikrobot camera (172.168.100.10) is reachable"
else
    echo "[WARN] Hikrobot camera (172.168.100.10) is not reachable via ping"
    echo "[INFO] This may be normal if camera is on a different network segment"
fi

# Check USB devices for cameras
echo "[INFO] Checking USB devices..."
if lsusb | grep -i hikvision >/dev/null 2>&1; then
    echo "[OK] Hikvision USB device detected"
    lsusb | grep -i hikvision
else
    echo "[INFO] No Hikvision USB devices found (this is normal for network cameras)"
fi

# Check for RealSense cameras
if lsusb | grep -i "Intel.*RealSense" >/dev/null 2>&1; then
    echo "[OK] RealSense USB device detected"
    lsusb | grep -i "Intel.*RealSense"
else
    echo "[INFO] No RealSense USB devices found"
fi

# Check video devices
echo "[INFO] Checking video devices..."
if ls -la /dev/video* 2>/dev/null; then
    echo "[OK] Video devices present"
else
    echo "[INFO] No video devices found"
fi

echo
echo "--------------------"
echo "🔧 Checking camera_aravis2 installation"
echo "--------------------"

# Check if camera_aravis2 is built and available
if [ -f /home/drims/static/drims2_ws/install/setup.bash ]; then
    source /home/drims/static/drims2_ws/install/setup.bash
    if command_exists ros2; then
        # Test if camera_aravis2 package is available
        if ros2 pkg list | grep -q camera_aravis2; then
            echo "[OK] camera_aravis2 package is available"
            
            # List available cameras (non-blocking, timeout after 10 seconds)
            echo "[INFO] Scanning for available cameras (timeout: 10s)..."
            timeout 10s ros2 run camera_aravis2 list_cameras || echo "[INFO] Camera scan completed or timed out"
        else
            echo "[WARN] camera_aravis2 package not found in ROS2 packages"
        fi
    fi
fi

# Check if camera_aravis2 node is built
if [ -f /home/drims/static/drims2_ws/install/camera_aravis2/lib/camera_aravis2/camera_aravis2 ]; then
    echo "[OK] camera_aravis2 node is built"
else
    echo "[WARN] camera_aravis2 node not found in build directory"
fi

echo
echo "--------------------"
echo "🌐 Checking network configuration"
echo "--------------------"

# Check network interfaces
echo "[INFO] Network interfaces:"
ip addr show | grep -E "inet (172\.168|192\.168|10\.)" | head -5

# Check if we can route to camera network
echo "[INFO] Routing table for camera network:"
ip route | grep 172.168 || echo "[INFO] No specific route for 172.168.0.0/16"

echo
echo "--------------------"
echo "📦 Checking ROS2 package availability"
echo "--------------------"

# Check essential ROS2 packages
if command_exists ros2; then
    essential_packages=(
        "realsense2_camera"
        "camera_aravis2" 
        "rviz2"
        "rqt_image_view"
        "image_transport"
    )
    
    available_packages=$(ros2 pkg list)
    
    for pkg in "${essential_packages[@]}"; do
        if echo "$available_packages" | grep -q "$pkg"; then
            echo "[OK] $pkg available"
        else
            echo "[WARN] $pkg not available"
        fi
    done
fi

echo
# Print script end banner
echo "===================="
echo "✅ Finished check_script.sh"
echo "===================="

# Keep the container running for further inspection if needed
echo
echo "💡 You can now run:"
echo "   ros2 run camera_aravis2 list_cameras"
echo "   ros2 run camera_aravis2 camera_aravis2_node --ros-args -p device:=\"172.168.100.10\""
echo "   ros2 launch realsense2_camera rs_launch.py"
echo