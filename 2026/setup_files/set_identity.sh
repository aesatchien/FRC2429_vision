#!/bin/bash
# Give this board its name and robot IP. Works on Raspberry Pi and Orange Pi.
#
#   sudo bash set_identity.sh <hostname> <ip>
#   sudo bash set_identity.sh frc-pi5-dual-arducam 10.24.29.12
#
# The hostname chooses BOTH config files at boot, so it must exist in both:
#   wpilib_frcjsons/<hostname>.json   and   a "<hostname>" entry under "hosts" in config/vision.json
# This script checks that before changing anything.

set -euo pipefail

SETUP_DIR="$(cd "$(dirname "$0")" && pwd)"     # .../2026/setup_files
REPO_DIR="$(dirname "$SETUP_DIR")"              # .../2026

if [ "$EUID" -ne 0 ]; then
    echo "Run with sudo:  sudo bash $0 <hostname> <ip>"
    exit 1
fi
if [ "$#" -ne 2 ]; then
    echo "Usage:   sudo bash $0 <hostname> <ip>"
    echo "Example: sudo bash $0 frc-pi5-dual-arducam 10.24.29.12"
    exit 1
fi

NEW_HOSTNAME="$1"
NEW_IP="$2"
OLD_HOSTNAME="$(hostname)"
FRC_JSON="$REPO_DIR/wpilib_frcjsons/$NEW_HOSTNAME.json"
VISION_JSON="$REPO_DIR/config/vision.json"

# --- 1. Check everything before changing anything ---

# runCamera loads wpilib_frcjsons/<hostname>.json...
if [ ! -f "$FRC_JSON" ]; then
    echo "ERROR: $FRC_JSON does not exist. Valid names:"
    ls "$REPO_DIR/wpilib_frcjsons" | sed 's/\.json$//'
    exit 1
fi

# ...and vision.json needs a host entry with the same name (a key like  "frc-pi5-dual-arducam": {  ).
KEY_PATTERN="\"$NEW_HOSTNAME\"[[:space:]]*:"
if ! grep -q "$KEY_PATTERN" "$VISION_JSON"; then
    echo "ERROR: no \"$NEW_HOSTNAME\" entry under \"hosts\" in $VISION_JSON"
    echo "Without it the board would run the 'default' profile, which has no cameras."
    exit 1
fi

# The IP must be on the robot network.
if [[ ! "$NEW_IP" =~ ^10\.24\.29\.[0-9]{1,3}$ ]]; then
    echo "ERROR: '$NEW_IP' is not a 10.24.29.x address."
    exit 1
fi

# setup.sh creates the 'ethernet' profile that holds the static IP.
if ! nmcli connection show ethernet > /dev/null 2>&1; then
    echo "ERROR: no NetworkManager profile named 'ethernet'. Run setup_files/setup.sh first."
    exit 1
fi

echo "--- Setting identity: $OLD_HOSTNAME -> $NEW_HOSTNAME, IP $NEW_IP ---"

# --- 2. Hostname ---
hostnamectl set-hostname "$NEW_HOSTNAME"

# Raspberry Pi OS Trixie images (Nov 2025+) set up with Raspberry Pi Imager:
# cloud-init re-applies the Imager hostname from user-data on EVERY boot.
# Changing it there too makes the new name stick.
CLOUD_USER_DATA="/boot/firmware/user-data"
if [ -f "$CLOUD_USER_DATA" ] && grep -q "^hostname:" "$CLOUD_USER_DATA"; then
    sed -i "s/^hostname:.*/hostname: $NEW_HOSTNAME/" "$CLOUD_USER_DATA"
    echo "Updated hostname in $CLOUD_USER_DATA (cloud-init)."
fi

# --- 3. /etc/hosts (without it, sudo warns "unable to resolve host") ---
if grep -q "^127\.0\.1\.1" /etc/hosts; then
    sed -i "s/^127\.0\.1\.1.*/127.0.1.1 $NEW_HOSTNAME/" /etc/hosts
else
    echo "127.0.1.1 $NEW_HOSTNAME" >> /etc/hosts
fi
# Orange Pi images also list the name on the ::1 line. Swap only the old name; keep the other aliases.
if [ "$OLD_HOSTNAME" != "$NEW_HOSTNAME" ] && [ "$OLD_HOSTNAME" != "localhost" ]; then
    sed -i "/^::1/s/\b$OLD_HOSTNAME\b/$NEW_HOSTNAME/" /etc/hosts
fi

# --- 4. Static IP ---
nmcli connection modify ethernet ipv4.addresses "$NEW_IP/24"
echo "Applying the new IP. If you're connected over this cable, SSH will drop - reconnect to $NEW_IP."
if ! nmcli connection up ethernet; then
    echo "Could not apply it now (cable unplugged?). It will apply on the next boot."
fi

echo "--- Done. Reboot to finish:  sudo reboot ---"
