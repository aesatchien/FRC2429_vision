WPILIB WAY - No longer that useful since they don't maintain it
======================
Deploying from desktop
======================

On the rPi web dashboard:

1) Make the rPi writable by selecting the "Writable" tab
2) In the rPi web dashboard Application tab, select the "Uploaded Python file"
   option for Application
3) Click "Browse..." and select the "multiCameraServer.py" file in
   your desktop project directory
4) Click Save

The application will be automatically started.  Console output can be seen by
enabling console output in the Vision Status tab.

========================
Deploying locally on rPi
========================

1) Copy multiCameraServer.py and runCamera to /home/pi
2) Run "./runInteractive" in /home/pi or "sudo svc -t /service/camera" to
   restart service.

# ------ 2025 0225
CJH WAY - from scratch
new py 5 - it keeps dying if you don't give it more than three amps.  jeez.
put on bookworm lite
plug in with monitor and enable ssh
run sudo raspi-config and set up ssh and wireless (but you can do with the imager
let raspi-config update stuff
sudo apt update
sudo apt upgrade
sudo apt install python3-pip
sudo apt install git
sudo apt-get install libgl1-mesa-glx

# ------

sudo apt install git
mkdir git  && cd git
git clone https://github.com/aesatchien/FRC2429_vision.git

unfortunately, because it is an externally managed environment, you HAVE to make a venv
python -m venv ~/robo2025

and then in the bin folder you can do source activate in the folder to have your venv and pip working

I tried python3 -m pip install robotpy[all] but that failed
needed 
python3 -m pip install --extra-index-url=https://wpilib.jfrog.io/artifactory/api/pypi/wpilib-python-release-2025/simple robotpy[all]

pip install matplotlib pandas scipy opencv-python

most of the following is now in a script - i need to do the other stuff above in the script too.
# ------ 
had to set the nmcli stuff - see the chatgpt thread and the files

sudo nmcli connection modify "Wired connection 1" ipv4.ignore-auto-dns yes
sudo nmcli connection modify "Wired connection 1" ipv4.dhcp-client-id ""
sudo nmcli connection modify "Wired connection 1" ipv4.dhcp-timeout 0


because i am in a venv, i need to source activate it.
# ------ this is in my .bashrc:

alias caminfo='ls -l /dev/v4l/by-id/'
alias gopy='cd ~/git/FRC2429_vision/2025/python_2025_multicam_2429'
alias startcams='sudo systemctl start runCamera.service'
alias stopcams='sudo systemctl stop runCamera.service'
alias stopwlan='sudo bash -c "echo \"blacklist brcmfmac\" > /etc/modprobe.d/disable-wifi.conf && reboot"'
alias startwlan='sudo rm /etc/modprobe.d/disable-wifi.conf && sudo reboot'

stop_wlan_fn() {
    sudo bash -c 'echo "blacklist brcmfmac" > /etc/modprobe.d/disable-wifi.conf && reboot'
}
source ~/robo2025/bin/activate

# --  use the caminfo alias to help set up the /boot/frc/json for this pi/camera combo


# ------  here is my /etc/systemd/system/runCamera.service
[Unit]
Description=Run Camera Script on Boot
After=network.target

[Service]
ExecStart=/home/pi/git/FRC2429_vision/2025/python_2025_multicam_2429/runCamera
WorkingDirectory=/home/pi/git/FRC2429_vision/2025/python_2025_multicam_2429
User=pi
Group=pi
Restart=always
StandardOutput=append:/home/pi/logs/runCamera.log
StandardError=append:/home/pi/logs/runCamera.err.log

[Install]
WantedBy=multi-user.target

# and after that
sudo systemctl daemon-reload
sudo systemctl enable runCamera.service
sudo systemctl start runCamera.service

you have to have it **enabled**

# ------  and here is the runCamera:
#!/bin/sh
### TYPE: upload-python
echo "Waiting 3 seconds..."
sleep 3
export PYTHONUNBUFFERED=1

# Activate the virtual environment
. /home/pi/robo2025/bin/activate  # Use "." instead of "source" for POSIX compliance

# Run the Python script inside the virtual environment
python3 /home/pi/git/FRC2429_vision/2025/python_2025_multicam_2429/multiCameraServer.py

 disable and renable wifi
remove:
sudo bash -c 'echo "blacklist brcmfmac" > /etc/modprobe.d/disable-wifi.conf && reboot'
turn on:
sudo rm /etc/modprobe.d/disable-wifi.conf && sudo reboot

bluetooth:
remove:
sudo bash -c 'echo "blacklist btbcm" > /etc/modprobe.d/disable-bluetooth.conf && echo "blacklist hci_uart" >> /etc/modprobe.d/disable-bluetooth.conf && reboot'
enable:
sudo rm /etc/modprobe.d/disable-bluetooth.conf && sudo reboot
