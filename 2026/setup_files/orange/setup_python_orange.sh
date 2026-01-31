# get the sources updated
sudo tee -a /etc/apt/sources.list <<'EOF'
deb http://ports.ubuntu.com/ubuntu-ports jammy main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports jammy-updates main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports jammy-backports main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports jammy-security main restricted universe multiverse
EOF


# 0) (Optional) fix apt typo if you had it
sudo apt-get update

# 1) install build deps needed to compile CPython via pyenv
sudo apt-get install -y \
  build-essential curl git ca-certificates \
  libssl-dev zlib1g-dev libbz2-dev \
  libreadline-dev libsqlite3-dev \
  libffi-dev liblzma-dev xz-utils \
  tk-dev uuid-dev libncursesw5-dev

# Install system dependencies for OpenCV
echo "Installing system dependencies..."
sudo apt-get update && sudo apt-get install -y libgl1 libglib2.0-0

# 2) install pyenv (user-local)
curl -fsSL https://pyenv.run | bash

# 3) enable pyenv in your shell (bash)
cat >> ~/.bashrc <<'EOF'
export PYENV_ROOT="$HOME/.pyenv"
export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init -)"
EOF
source ~/.bashrc

# 4) install a real Python 3.11 (pick the patch you want; example shown)
pyenv install 3.11.12
pyenv global 3.11.12
python --version

# 5) create venv in your home dir named "robo2025"
rm -rf ~/robo2025
python -m venv ~/robo2025
source ~/robo2025/bin/activate

# 6) upgrade pip toolchain in the venv
python -m pip install -U pip setuptools wheel
python -m pip --version
python --version

# 7) install RobotPy 2025 from WPILib’s index
python3 -m pip install --extra-index-url=https://wpilib.jfrog.io/artifactory/api/pypi/wpilib-python-release-2025/simple   "pyntcore==2025.2.1.1" "robotpy-apriltag==2025.2.1.1" "robotpy-cscore==2025.2.1.1"
python3 -m pip install opencv-python

# 8) quick verification
python -m pip show pyntcore robotpy-wpimath
python - <<'EOF'
from ntcore import NetworkTableInstance
import wpimath
print("NTCore OK:", NetworkTableInstance.getDefault() is not None)
print("wpimath OK:", wpimath.__file__)
EOF
