#!/bin/bash

# Check for root privileges
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (use sudo)"
  exit 1
fi

# Check arguments
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <NEW_HOSTNAME> <NEW_IP>"
    echo "Example: $0 frc-pi5-dual-arducam 10.24.29.12"
    exit 1
fi

NEW_HOSTNAME=$1
NEW_IP=$2
NM_FILE="/etc/NetworkManager/system-connections/ethernet.nmconnection"

echo "--- Setting Identity ---"
echo "Hostname: $NEW_HOSTNAME"
echo "IP Address: $NEW_IP"

# 1. Set Hostname
echo "Setting hostname..."
hostnamectl set-hostname "$NEW_HOSTNAME"

# 2. Update /etc/hosts (Map 127.0.1.1 to new hostname)
echo "Updating /etc/hosts..."
sed -i "s/^127\.0\.1\.1.*/127.0.1.1 $NEW_HOSTNAME/" /etc/hosts

# 3. Update Static IP
if [ -f "$NM_FILE" ]; then
    echo "Updating IP in NetworkManager configuration..."
    
    # Ensure strict permissions (Root read/write only)
    chmod 600 "$NM_FILE"
    
    # Replace the addresses line (Assumes /24 subnet)
    sed -i "s|^addresses=.*|addresses=$NEW_IP/24|" "$NM_FILE"
    
    # Reload and Restart
    echo "Applying network changes..."
    nmcli connection reload
    nmcli connection down ethernet && nmcli connection up ethernet
else
    echo "WARNING: $NM_FILE not found! Skipping IP update."
fi

echo "Done! You may need to reboot for all changes to fully take effect."