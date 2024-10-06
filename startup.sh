echo Running startup

# Make venv
python3 -m venv .venv
echo venv created

# Activate venv
source .venv/bin/activate
echo venv activated

# Upgrade pip
pip install --upgrade pip
echo pip upgraded

# Install dependencies
pip install -r requirements.txt
echo dependencies installed

# Linux dependencies
sudo apt update && sudo apt install ffmpeg libsm6 libxext6 -y
sudo apt install xcb