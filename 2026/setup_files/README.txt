FRC 2429 Vision Setup Files
===========================

This folder contains the templates and scripts used by `setup_pi.sh` to configure a new coprocessor.

Files:
- bashrc_additions: Aliases appended to ~/.bashrc
- runCamera.service: Systemd service template (paths are updated by setup script)
- runCamera: Shell script wrapper to launch the python code

To setup a new Pi:
1. Run `../setup_pi.sh` from the parent directory.