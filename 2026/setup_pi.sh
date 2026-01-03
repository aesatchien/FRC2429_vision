#!/bin/bash

# FRC 2429 Vision Coprocessor Setup Script
# Supports Raspberry Pi 4/5 (Bookworm) and Orange Pi 5 (Armbian/Ubuntu)
# Run this script as the 'pi' (or primary) user, NOT as root (it will ask for sudo when needed).

set -e  # Exit immediately if a command exits with a non-zero status.

# --- Configuration ---
USER_HOME="/home/$(whoami)"
REPO_DIR="$USER_HOME/git/FRC2429_vision"
VENV_DIR="$USER_HOME/robo2025"
SETUP_FILES_DIR="$REPO_DIR/2026/setup_files"
CURRENT_YEAR_DIR="$REPO_DIR/2026"

# --- Helper Functions ---

log() {
    echo -e "\n\033[1;32m[SETUP] $1\033[0m"
}

check_internet() {
    log "Checking internet connection..."
    if ping -q -c 1 -W 1 google.com >/dev/null; then
        echo "Internet is reachable."
    else
        echo "Error: No internet connection. Please connect via Ethernet or WiFi."
        exit 1
    fi
}

system_update() {
    log "Updating system packages..."
    sudo apt update
    sudo apt upgrade -y
    
    log "Installing system dependencies..."
    # Common dependencies for OpenCV, v4l2, and build tools
    sudo apt install -y git python3-pip python3-venv v4l-utils libgl1-mesa-glx libglib2.0-0
    
    # Network manager tools (for static IP setup later if needed)
    sudo apt install -y network-manager
}

setup_venv() {
    log "Setting up Python Virtual Environment at $VENV_DIR..."
    
    if [ ! -d "$VENV_DIR" ]; then
        python3 -m venv "$VENV_DIR"
        echo "Virtual environment created."
    else
        echo "Virtual environment already exists."
    fi

    # Activate venv to install packages
    source "$VENV_DIR/bin/activate"

    log "Upgrading pip..."
    pip install --upgrade pip

    log "Installing Python libraries..."
    # RobotPy requires a specific extra index URL for 2025
    # Note: Adjust the year in the URL if needed for 2026 beta/release
    pip install --extra-index-url=https://wpilib.jfrog.io/artifactory/api/pypi/wpilib-python-release-2025/simple robotpy[all]
    
    # Standard libraries
    pip install matplotlib pandas scipy opencv-python
    
    echo "Python dependencies installed."
}

configure_network() {
    log "Configuring Network Manager (preventing DNS/DHCP hang)..."
    # This prevents the Pi from waiting for a DHCP lease on the FRC radio network, which speeds up boot
    
    # Check if "Wired connection 1" exists, otherwise try to find the active ethernet connection
    CONN_NAME="Wired connection 1"
    if ! nmcli connection show "$CONN_NAME" >/dev/null 2>&1; then
        echo "Connection '$CONN_NAME' not found. Listing connections:"
        nmcli connection show
        echo "Skipping specific nmcli modifications. Please verify connection name."
    else
        sudo nmcli connection modify "$CONN_NAME" ipv4.ignore-auto-dns yes
        sudo nmcli connection modify "$CONN_NAME" ipv4.dhcp-client-id ""
        sudo nmcli connection modify "$CONN_NAME" ipv4.dhcp-timeout 1
        echo "Network settings applied to $CONN_NAME."
    fi
}

setup_bashrc() {
    log "Configuring .bashrc aliases..."
    BASHRC="$USER_HOME/.bashrc"
    ADDITIONS_FILE="$SETUP_FILES_DIR/bashrc_additions"

    if [ -f "$ADDITIONS_FILE" ]; then
        if grep -q "FRC 2429 Setup" "$BASHRC"; then
            echo ".bashrc already contains FRC setup. Skipping append."
        else
            echo "" >> "$BASHRC"
            echo "# --- FRC 2429 Setup ---" >> "$BASHRC"
            cat "$ADDITIONS_FILE" >> "$BASHRC"
            echo "Aliases added to .bashrc."
        fi
    else
        echo "Warning: $ADDITIONS_FILE not found. Skipping aliases."
    fi
}

setup_service() {
    log "Setting up systemd service..."
    
    SERVICE_FILE="$SETUP_FILES_DIR/runCamera.service"
    SYSTEM_SERVICE="/etc/systemd/system/runCamera.service"
    RUN_SCRIPT="$SETUP_FILES_DIR/runCamera"
    TARGET_RUN_SCRIPT="$CURRENT_YEAR_DIR/runCamera"

    # 1. Copy the run script to the working directory and make executable
    if [ -f "$RUN_SCRIPT" ]; then
        cp "$RUN_SCRIPT" "$TARGET_RUN_SCRIPT"
        chmod +x "$TARGET_RUN_SCRIPT"
        # Ensure the script points to the correct venv and python file
        # We use sed to dynamically update paths in the runCamera script if needed, 
        # but for now we assume the repo structure is constant.
    else
        echo "Error: $RUN_SCRIPT not found."
        return
    fi

    # 2. Copy and enable the service file
    if [ -f "$SERVICE_FILE" ]; then
        # Update paths in service file to match current user/path
        # This allows installing on 'pi', 'orangepi', 'ubuntu', etc.
        sed "s|/home/pi|$USER_HOME|g" "$SERVICE_FILE" | sudo tee "$SYSTEM_SERVICE" > /dev/null
        
        # Also update User/Group if not 'pi'
        CURRENT_USER=$(whoami)
        sudo sed -i "s|User=pi|User=$CURRENT_USER|g" "$SYSTEM_SERVICE"
        sudo sed -i "s|Group=pi|Group=$CURRENT_USER|g" "$SYSTEM_SERVICE"

        sudo systemctl daemon-reload
        sudo systemctl enable runCamera.service
        echo "Service enabled. It will start on reboot."
        
        # Create log directory
        mkdir -p "$USER_HOME/logs"
    else
        echo "Error: $SERVICE_FILE not found."
    fi
}

# --- Main Execution ---

check_internet
system_update
setup_venv
configure_network
setup_bashrc
setup_service

log "Setup Complete!"
echo "Please reboot your system to apply all changes."
echo "  sudo reboot"