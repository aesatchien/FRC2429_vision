#!/bin/bash
# Turn the onboard Wi-Fi off (competition) or back on (shop), then reboot.
#   bash wifi_power.sh off
#   bash wifi_power.sh on
# (The aliases stopwlan / startwlan call this.)

set -euo pipefail

ACTION="${1:-}"
BLACKLIST_FILE="/etc/modprobe.d/disable-wifi.conf"

BOARD=""
if [ -r /proc/device-tree/model ]; then
    BOARD="$(tr -d '\0' < /proc/device-tree/model)"
fi

# The Pi's config.txt moved to /boot/firmware in Bookworm.
if [ -f /boot/firmware/config.txt ]; then
    PI_CONFIG="/boot/firmware/config.txt"
else
    PI_CONFIG="/boot/config.txt"
fi

case "$ACTION" in
    off)
        if [[ "$BOARD" == *"Raspberry Pi"* ]]; then
            # Firmware switch: the Wi-Fi chip is never started.
            if ! grep -qx "dtoverlay=disable-wifi" "$PI_CONFIG"; then
                echo "dtoverlay=disable-wifi" | sudo tee -a "$PI_CONFIG" > /dev/null
            fi
            echo "dtoverlay=disable-wifi is set in $PI_CONFIG"
        else
            # Other boards: block the Wi-Fi driver. Read its real name instead of guessing (bcmdhd, brcmfmac...).
            if [ ! -e /sys/class/net/wlan0 ]; then
                echo "No wlan0 interface found - Wi-Fi may already be off. Nothing changed."
                exit 1
            fi
            DRIVER_PATH="$(readlink -f /sys/class/net/wlan0/device/driver)"
            DRIVER_NAME="$(basename "$DRIVER_PATH")"
            echo "blacklist $DRIVER_NAME" | sudo tee "$BLACKLIST_FILE" > /dev/null
            echo "Blacklisted Wi-Fi driver: $DRIVER_NAME"
        fi
        ;;
    on)
        if [ -f "$PI_CONFIG" ]; then
            sudo sed -i '/^dtoverlay=disable-wifi$/d' "$PI_CONFIG"
        fi
        sudo rm -f "$BLACKLIST_FILE"
        echo "Wi-Fi blocks removed."
        ;;
    *)
        echo "Usage: bash $0 off|on"
        exit 1
        ;;
esac

echo "Rebooting..."
sudo reboot
