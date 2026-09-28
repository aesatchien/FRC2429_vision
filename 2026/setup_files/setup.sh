#!/bin/bash
# FRC 2429 vision coprocessor setup - Raspberry Pi 4/5 and Orange Pi 5.
# Works on Raspberry Pi OS Bookworm/Trixie and Ubuntu 22.04/24.04.
# Safe to run again at any time; every step checks before it changes anything.
#
#   1. Clone the repo, then run as your normal user (NOT with sudo):
#          bash setup_files/setup.sh
#   2. Name the board and give it its robot IP:
#          sudo bash setup_files/set_identity.sh <hostname> <ip>
#   3. sudo reboot

set -euo pipefail   # stop on the first error, unset variable, or failed pipe

# --- Paths: worked out from where this script lives, so no /home/pi assumptions ---
SETUP_DIR="$(cd "$(dirname "$0")" && pwd)"    # .../2026/setup_files
REPO_DIR="$(dirname "$SETUP_DIR")"             # .../2026
RUN_USER="$(id -un)"                           # the user the service will run as
VENV_DIR="$HOME/robo2025"                      # name kept so existing Pis keep working (also in runCamera)
PYENV_PY_VERSION="3.11.12"                     # only used when the system Python is too old
DEFAULT_IP="10.24.29.13"                       # placeholder; set_identity.sh sets the real one

log() {
    echo -e "\n\033[1;32m[SETUP] $1\033[0m"
}

# --- Must not be root: $HOME and the service user would both be wrong ---
if [ "$EUID" -eq 0 ]; then
    echo "Run as your normal user, not with sudo. The script asks for sudo when it needs it."
    exit 1
fi

detect_system() {
    log "Detecting board and OS..."
    . /etc/os-release                          # sets ID (debian/ubuntu) and VERSION_CODENAME
    BOARD="unknown"
    if [ -r /proc/device-tree/model ]; then
        BOARD="$(tr -d '\0' < /proc/device-tree/model)"   # e.g. "Raspberry Pi 5 Model B Rev 1.0"
    fi
    echo "Board: $BOARD"
    echo "OS:    ${ID:-?} ${VERSION_CODENAME:-?}"
    echo "User:  $RUN_USER    Repo: $REPO_DIR"
}

check_internet() {
    log "Checking internet..."
    # Fetch a web page instead of ping: many school networks block ping.
    if curl -sfI --max-time 10 https://pypi.org > /dev/null; then
        echo "Internet is reachable."
    else
        echo "ERROR: cannot reach pypi.org. Connect via Ethernet or Wi-Fi and try again."
        exit 1
    fi
}

install_system_packages() {
    log "Installing system packages..."
    sudo apt-get update
    sudo apt-get upgrade -y
    # Only package names that exist on every OS we support.
    # (libgl1-mesa-glx is gone in Trixie; we don't need it because OpenCV is the headless build.)
    sudo apt-get install -y git curl python3-venv v4l-utils network-manager
}

install_pyenv_python() {
    echo "System Python is older than 3.11, so building Python $PYENV_PY_VERSION with pyenv (10-30 min)."
    # Build tools for compiling Python. libncurses-dev is the name that exists on every release.
    sudo apt-get install -y build-essential libssl-dev zlib1g-dev libbz2-dev libreadline-dev \
        libsqlite3-dev libffi-dev liblzma-dev tk-dev uuid-dev libncurses-dev xz-utils
    if [ ! -d "$HOME/.pyenv" ]; then
        curl -fsSL https://pyenv.run | bash
    fi
    # Call pyenv by its full path - no need to reload .bashrc.
    # -s = skip if this version is already built (otherwise pyenv stops and asks).
    "$HOME/.pyenv/bin/pyenv" install -s "$PYENV_PY_VERSION"
}

choose_python() {
    log "Choosing a Python for the venv..."
    # RobotPy's 64-bit Pi wheels need Python 3.11 or newer.
    # Bookworm (3.11), Trixie (3.13) and Ubuntu 24.04 (3.12) are fine; Ubuntu 22.04 (3.10) is not.
    SYSTEM_PY_IS_NEW_ENOUGH="$(python3 -c 'import sys; print(sys.version_info >= (3, 11))')"
    if [ "$SYSTEM_PY_IS_NEW_ENOUGH" = "True" ]; then
        PYTHON="python3"
    else
        install_pyenv_python
        PYTHON="$HOME/.pyenv/versions/$PYENV_PY_VERSION/bin/python"
    fi
    echo "Using $($PYTHON --version)"
}

setup_venv() {
    log "Setting up the Python venv at $VENV_DIR..."
    # Reuse an existing venv. Never delete it: the camera service may be using it.
    if [ ! -x "$VENV_DIR/bin/python" ]; then
        "$PYTHON" -m venv "$VENV_DIR"
    fi
    VENV_PY="$VENV_DIR/bin/python"
    "$VENV_PY" -m pip install --upgrade pip
    # Exact versions live in requirements-pi.txt so every board gets the same thing.
    "$VENV_PY" -m pip install -r "$SETUP_DIR/requirements-pi.txt"
    # Import test: find a broken install now, not at the first match.
    "$VENV_PY" -c "import cv2, ntcore, cscore, robotpy_apriltag, wpimath; print('Imports OK. OpenCV', cv2.__version__)"
}

install_reboot_permission() {
    log "Allowing the vision service to reboot the board..."
    # Since the 2026-04-13 Raspberry Pi OS image, sudo asks for a password by default,
    # so the automatic reboot after repeated camera failures would silently fail.
    # This rule allows ONLY reboot without a password.
    RULE="$RUN_USER ALL=(root) NOPASSWD: /usr/sbin/reboot, /sbin/reboot"
    RULE_FILE="/etc/sudoers.d/010-vision-reboot"
    TMP_RULE="$(mktemp)"
    echo "$RULE" > "$TMP_RULE"
    # Check the syntax BEFORE installing - a broken sudoers file can lock you out of sudo.
    sudo visudo -cf "$TMP_RULE"
    sudo install -m 440 -o root -g root "$TMP_RULE" "$RULE_FILE"
    rm -f "$TMP_RULE"
    echo "Installed: $RULE"
}

setup_ethernet() {
    log "Setting up the static 'ethernet' profile..."
    # Use the first wired interface: eth0 on a Pi, enP3p49s0 or enP4p65s0 on an Orange Pi 5 Plus.
    # To choose a different port:  ETH_IF=enP4p65s0 bash setup_files/setup.sh
    WIRED_DEVICES="$(nmcli -t -f DEVICE,TYPE device | grep ':ethernet$' || true)"
    FIRST_WIRED="$(echo "$WIRED_DEVICES" | head -n 1 | cut -d: -f1)"
    ETH_IF="${ETH_IF:-$FIRST_WIRED}"
    if [ -z "$ETH_IF" ]; then
        echo "ERROR: no wired interface found. Run 'nmcli device' to list them."
        exit 1
    fi
    echo "Wired interface: $ETH_IF"

    if nmcli connection show ethernet > /dev/null 2>&1; then
        # Keep the existing IP; set_identity.sh is the tool for changing it.
        echo "Profile 'ethernet' already exists - keeping its IP."
        sudo nmcli connection modify ethernet connection.interface-name "$ETH_IF"
    else
        sudo nmcli connection add type ethernet con-name ethernet ifname "$ETH_IF" \
            ipv4.method manual ipv4.addresses "$DEFAULT_IP/24" ipv4.gateway 10.24.29.1 \
            ipv4.dns "8.8.8.8 8.8.4.4"
    fi
    # Priority 100 beats any automatic DHCP profile, so the static IP always wins at boot.
    sudo nmcli connection modify ethernet connection.autoconnect yes connection.autoconnect-priority 100

    # The old setup_pi.sh saved a DHCP profile for the same port. Turn off its autoconnect
    # (we don't delete it: that would drop your SSH session if you're using it right now).
    if nmcli connection show "Wired connection 1" > /dev/null 2>&1; then
        sudo nmcli connection modify "Wired connection 1" connection.autoconnect no
        echo "Disabled autoconnect on 'Wired connection 1'."
    fi
    # Not activated now (that could drop your SSH session); it takes effect on reboot.
}

setup_shop_wifi() {
    log "Shop Wi-Fi (FRC-2429)..."
    if nmcli connection show FRC-2429 > /dev/null 2>&1; then
        echo "Already set up."
        return
    fi
    # The password is typed here and never stored in git.
    read -rsp "FRC-2429 Wi-Fi password (press Enter to skip): " WIFI_PASSWORD
    echo
    if [ -z "$WIFI_PASSWORD" ]; then
        echo "Skipped."
        return
    fi
    sudo nmcli connection add type wifi con-name FRC-2429 ssid FRC-2429 \
        wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$WIFI_PASSWORD"
}

setup_bashrc() {
    log "Adding the vision shortcuts to ~/.bashrc..."
    # We add ONE line that loads bashrc_additions from the repo, so a git pull updates the aliases.
    SOURCE_LINE=". \"$SETUP_DIR/bashrc_additions\"   # FRC 2429 vision shortcuts"
    if grep -qF "$SETUP_DIR/bashrc_additions" "$HOME/.bashrc"; then
        echo "Already there."
    else
        echo "" >> "$HOME/.bashrc"
        echo "$SOURCE_LINE" >> "$HOME/.bashrc"
        echo "Added: $SOURCE_LINE"
    fi
}

install_service() {
    log "Installing runCamera.service..."
    TEMPLATE="$SETUP_DIR/runCamera.service.template"
    UNIT_FILE="/etc/systemd/system/runCamera.service"
    # Fill in this board's user and repo path.
    sed -e "s|@USER@|$RUN_USER|g" -e "s|@REPO@|$REPO_DIR|g" "$TEMPLATE" | sudo tee "$UNIT_FILE" > /dev/null
    sudo systemctl daemon-reload
    # Enable only. It starts on the next boot, after set_identity.sh has named the board.
    sudo systemctl enable runCamera.service
}

# --- Main ---
detect_system
check_internet
install_system_packages
choose_python
setup_venv
install_reboot_permission
setup_ethernet
setup_shop_wifi
setup_bashrc
install_service

log "Setup complete!"
echo "Next:  sudo bash $SETUP_DIR/set_identity.sh <hostname> <ip>"
echo "Then:  sudo reboot"
