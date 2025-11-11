#!/bin/bash

# Setup OnRobot network configuration
echo "Setting up OnRobot network configuration..."

# Configure network interface for OnRobot gripper (adjust interface name as needed)
INTERFACE="eth0"

# Set static IP for OnRobot gripper communication
sudo ip addr add 192.168.1.100/24 dev $INTERFACE || true

# Enable the interface
sudo ip link set $INTERFACE up

echo "OnRobot network configuration completed"
echo "Interface: $INTERFACE"
echo "IP Address: 192.168.1.100/24"