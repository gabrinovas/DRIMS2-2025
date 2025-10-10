#!/bin/bash

# Enhanced OnRobot network detection and setup script

ONROBOT_IP="192.168.1.1"   # Default IP address of the OnRobot Compute Box
ONROBOT_PORT="502"         # Default Modbus port for OnRobot
INTERFACE=""

# Function to detect the network interface connected to the OnRobot subnet (192.168.1.x)
detect_onrobot_interface() {
    # Look for interface with 192.168.1.x address
    ip -o -4 addr show | grep "192.168.1." | awk '{print $2}' | head -1
}

# Function to check connectivity to the OnRobot Compute Box
check_onrobot_connection() {
    echo "Checking OnRobot Compute Box at $ONROBOT_IP:$ONROBOT_PORT..."
    
    # Detect interface
    INTERFACE=$(detect_onrobot_interface)
    
    if [ -n "$INTERFACE" ]; then
        echo "✅ Detected OnRobot network interface: $INTERFACE"
        
        # Test basic connectivity (ping the OnRobot IP)
        if ping -c 1 -W 2 $ONROBOT_IP >/dev/null 2>&1; then
            # Test if Modbus port is open
            if timeout 2 bash -c "echo > /dev/tcp/$ONROBOT_IP/$ONROBOT_PORT" 2>/dev/null; then
                echo "✅ HARDWARE MODE: OnRobot Compute Box connected via $INTERFACE"
                return 0  # Success - hardware available
            else
                echo "⚠️  Compute Box reachable but Modbus port $ONROBOT_PORT not accessible"
                echo "   This may be normal if the gripper is not powered on or configured"
                return 1  # Partial failure
            fi
        else
            echo "❌ SIMULATION MODE: OnRobot Compute Box not reachable"
            return 2  # Hardware not available
        fi
    else
        echo "❌ SIMULATION MODE: No OnRobot network interface detected"
        return 2  # Hardware not available
    fi
}

# Function to set environment variables based on detection result
set_environment() {
    local status=$1
    
    case $status in
        0)
            export ONROBOT_MODE="hardware"
            export ONROBOT_IP=$ONROBOT_IP
            export ONROBOT_SIMULATION="false"
            ;;
        1|2)
            export ONROBOT_MODE="simulation" 
            export ONROBOT_IP="127.0.0.1"
            export ONROBOT_SIMULATION="true"
            ;;
    esac
    
    echo "Environment: ONROBOT_MODE=$ONROBOT_MODE, ONROBOT_IP=$ONROBOT_IP"
}

# Main execution logic
main() {
    case "${1:-}" in
        "check")
            # Only check and set environment, then exit with status
            check_onrobot_connection
            status=$?
            set_environment $status
            exit $status
            ;;
        "status")
            # Check, set environment, and print a status string
            check_onrobot_connection
            status=$?
            set_environment $status
            case $status in
                0) echo "HARDWARE_READY" ;;
                1) echo "PARTIAL_CONNECTION" ;;
                2) echo "SIMULATION_MODE" ;;
            esac
            ;;
        *)
            # Default: check and set environment, no exit or status print
            check_onrobot_connection
            status=$?
            set_environment $status
            ;;
    esac
}

# Run the main function with any provided argument
main "$@"