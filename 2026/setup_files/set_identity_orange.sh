#!/bin/bash

# Check for root privileges
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <NEW_HOSTNAME> <NEW_IP>"
    exit 1
fi

NEW_HOSTNAME=$1
NEW_IP=$2

echo "--- Updating Identity ---"

# 1. Set Hostname
hostnamectl set-hostname "$NEW_HOSTNAME"

# 2. Update /etc/hosts by targeting the IP lines, not the name strings
echo "Updating /etc/hosts..."

# Replace the entire 127.0.1.1 line
sed -i "s/^127\.0\.1\.1.*/127.0.1.1 $NEW_HOSTNAME/" /etc/hosts

# Replace the name in the ::1 line (preserves localhost and ip6 aliases)
sed -i "/^::1/s/localhost [^ ]*/localhost $NEW_HOSTNAME/" /etc/hosts

# 3. Find and Update IP via nmcli
CON_NAME=$(nmcli -t -f NAME,TYPE connection show --active | grep -i "ethernet" | cut -d: -f1 | head -n 1)

if [ -z "$CON_NAME" ]; then
    CON_NAME=$(nmcli -t -f NAME,TYPE connection show | grep -i "ethernet" | cut -d: -f1 | head -n 1)
fi

if [ -n "$CON_NAME" ]; then
    echo "Updating IP for: $CON_NAME"
    nmcli connection modify "$CON_NAME" ipv4.addresses "$NEW_IP/24" ipv4.method manual
    nmcli connection up "$CON_NAME"
else
    echo "ERROR: No Ethernet connection found."
    exit 1
fi

echo "--- Done ---"