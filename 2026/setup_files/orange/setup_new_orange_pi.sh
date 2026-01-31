#!/bin/bash
# Setup script to configure a new Raspberry Pi

echo "Starting setup..."

# Ensure the script is running from the correct directory
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Copy .bashrc to home directory
echo "Updating .bashrc..."
cp "$SCRIPT_DIR/.bashrc_orangepi" ~/.bashrc

# Copy ethernet.nmconnection to NetworkManager configuration - set eth0 t0 10.24.29.13
echo "Setting up Ethernet static IP configuration..."
sudo cp "$SCRIPT_DIR/ethernet.nmconnection_orange" /etc/NetworkManager/system-connections/ethernet.nmconnection
sudo chmod 600 /etc/NetworkManager/system-connections/ethernet.nmconnection
sudo nmcli connection reload

# Ensure the runCamera script is executable
echo "Making runCamera script executable..."
chmod +x "$SCRIPT_DIR/runCamera"

# Copy and enable runCamera.service
echo "Installing runCamera.service..."
sudo cp "$SCRIPT_DIR/runCamera_orange.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable runCamera.service
sudo systemctl start runCamera.service

# Copy and set up Wi-Fi configuration for FRC-2429
#echo "Adding Wi-Fi profile for FRC-2429..."
#sudo cp "$SCRIPT_DIR/FRC-2429.nmconnection" /etc/NetworkManager/system-connections/
#sudo chmod 600 /etc/NetworkManager/system-connections/FRC-2429.nmconnection
#sudo nmcli connection reload

echo "Setup complete!"
