FRC 2429 Vision - Coprocessor Setup
===================================
One set of scripts for Raspberry Pi 4/5 and Orange Pi 5
(Raspberry Pi OS Bookworm/Trixie, Ubuntu 22.04/24.04).

NEW BOARD
1. Flash the OS. In Raspberry Pi Imager, leave the hostname blank (set_identity.sh sets it).
2. git clone the repo to ~/git/FRC2429_vision   (any folder works)
3. bash setup_files/setup.sh                            (normal user, NOT sudo; safe to re-run)
4. sudo bash setup_files/set_identity.sh <hostname> <ip>
       e.g. sudo bash setup_files/set_identity.sh frc-pi5-dual-arducam 10.24.29.12
5. sudo reboot
6. logcams        (live service log; Ctrl-C to quit)

EXISTING BOARD (moving to these scripts)
    git pull, then steps 3 and 5. Your venv, IP and hostname are kept.

FILES
  setup.sh                     Installs packages, venv, reboot permission, network profile, service.
  set_identity.sh              Sets hostname + static IP. Refuses names missing from the config files.
  runCamera                    What the service runs. Picks wpilib_frcjsons/<hostname>.json.
  runCamera.service.template   Service file; setup.sh fills in the user and folder.
  requirements-pi.txt          Exact Python package versions.
  bashrc_additions             Aliases (gopy, startcams, stopcams, logcams, stopwlan, startwlan...).
  wifi_power.sh                Wi-Fi off/on for competition (called by stopwlan/startwlan).
  remove_runcamera.sh          Uninstall the service.
  .gitattributes               Keeps Linux line endings on these scripts when committed from Windows.

RULE: a hostname must exist in BOTH places or the cameras won't start:
  wpilib_frcjsons/<hostname>.json   and   "<hostname>" under "hosts" in config/vision.json


WHAT CHANGED (Sept 2026) AND WHY
- One setup.sh replaces setup_pi.sh, raspberry/setup_new_pi.sh, orange/setup_new_orange_pi.sh
  and orange/setup_python_orange.sh. The old scripts pointed at files that had moved, so the
  service was never installed and ~/logs was never created.
- setup.sh detects the board and OS instead of assuming them. It only builds Python with
  pyenv when the system Python is older than 3.11 (Ubuntu 22.04).
- Dropped libgl1-mesa-glx and libncursesw5-dev: they don't exist on Trixie / Ubuntu 24.04,
  and apt refused the whole install line. OpenCV is now the headless build, which needs no libGL.
- Python packages are pinned in requirements-pi.txt. Unpinned installs had started pulling
  OpenCV 5.0 and RobotPy 2026, and robotpy[all] installed 20 packages we don't use.
- Reboot permission: since the 2026-04-13 Raspberry Pi OS image, sudo needs a password by
  default, so the automatic reboot after repeated camera failures silently failed. setup.sh
  installs a rule that allows ONLY reboot, and the Python code now logs if the reboot fails.
- set_identity.sh (one version for both boards):
    * checks the hostname exists in both config files before changing anything;
    * also writes the hostname into /boot/firmware/user-data, because on Trixie images
      cloud-init otherwise resets it on every boot;
    * sets the IP with nmcli.
- Network: one 'ethernet' profile at priority 100, so an automatic DHCP profile can't take over.
- Service: runs the tracked runCamera directly (a git pull updates it), logs to the journal
  (no ~/logs folder, rotated automatically), and waits for the network properly.
- Wi-Fi off/on: the old stopwlan alias was defined twice and blocked the Orange Pi driver on
  Raspberry Pis. wifi_power.sh uses dtoverlay=disable-wifi on a Pi and looks up the real
  driver name on other boards.
- The shop Wi-Fi password is typed during setup, no longer stored in git.
  (It was in a public repo: change the Wi-Fi password.)
- Fixed three hostnames that existed in only one config file:
  frc-pi4-dual-c920, frc-pi4-single-c920, frc-orange5-single-arducam.
