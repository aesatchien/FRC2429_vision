# 1. Stop the running service
sudo systemctl stop runCamera.service

# 2. Disable it so it doesn't try to start on boot
sudo systemctl disable runCamera.service

# 3. Delete the service file
sudo rm /etc/systemd/system/runCamera.service

# 4. Reload systemd to recognize the file is gone
sudo systemctl daemon-reload

# 5. Reset any failed state associated with the removed service
sudo systemctl reset-failed
