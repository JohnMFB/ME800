# ME800  
Tello Drone PID/L1 Adaptive Controller Project

## Prerequisites  
- **Windows**: Install VcXsrv (X Server) to display Linux GUI apps on Windows
- **WSL1**: Ubuntu distribution with networking bridged to the Tello hotspot.

## 1. Configure X Server forwarding  
1. **Launch VcXsrv (XLaunch)** on Windows with defaults (Display 0, clipboard enabled, no access control) 
2. In WSL1, set your DISPLAY environment variable and verify it:  
   ```bash
   export DISPLAY=:0
   echo $DISPLAY   # should print ":0"

## Create/export runtime directory for GUI Apps
mkdir -p /tmp/runtime-$(whoami)
export XDG_RUNTIME_DIR=/tmp/runtime-$(whoami)
echo $XDG_RUNTIME_DIR

### Test Video Stream Playback With Tello Connected
ffplay -i udp://@:11111

## Install WSL1 Dependencies
sudo apt update
sudo apt upgrade -y

# Python core + venv
sudo apt install -y python3 python3-pip python3-venv  

# Video & X11 libraries
sudo apt install -y ffmpeg libsm6 libxext6           

# Build tools & headers
sudo apt install -y build-essential python3-dev     

# Image codec support
sudo apt install -y libjpeg-dev zlib1g-dev         

## Create & Activate Python Virtual Env

cd ~/projects/ME800
python3 -m venv Tello                              
source Tello/bin/activate                            
pip install --upgrade pip

## Install Python Packages

pip install opencv-python                             

# CPU‑only PyTorch + audio support
pip install torch torchvision torchaudio \
  --extra-index-url https://download.pytorch.org/whl/cpu  

pip install -U ultralytics                             


## Installation Verification

# Check PyTorch CPU-only build
python3 -c "import torch; print(torch.__version__, 'CPU only:', not torch.cuda.is_available())"
# e.g. prints "2.6.0+cpu CPU only: True" 
# Check YOLOv8 model loading
python3 -c "from ultralytics import YOLO; print(YOLO('yolov8n.pt'))"
# Should download & initialize the ‘n’ model, printing its summary 


# Tello Wi‑Fi Setup
Hotspot SSID: Tello-CBF1B9

Default IP: 192.168.10.1


<details>
<summary> OLD README</summary>

# ME800
Tello Drone PID/L1 Adaptive Controller Project

#### Setup
(SET TO WSL 1)
- WSL, SSH, VSCode, Git Clone, ...
- Python
Creating
https://sourceforge.net/projects/vcxsrv/files/latest/download
- Check Lighter Security
export DISPLAY=:0
echo $DISPLAY

mkdir -p /tmp/runtime-$(whoami)
export XDG_RUNTIME_DIR=/tmp/runtime-$(whoami)
echo $XDG_RUNTIME_DIR

ffplay -i udp://@:11111 (Testing Camera)

WSL Dependencies (Install in each device):
sudo apt install python3 python3-pip python3-venv -y
sudo apt install ffmpeg libsm6 libxext6
sudo apt install -y build-essential python3-dev
sudo apt install -y libjpeg-dev zlib1g-dev

sudo apt update
sudo apt upgrade

Github Env:
python3 -m venv Tello
Activating Tello:
source Tello/bin/activate

pip install --upgrade pip
Python Packages Within Shared env:

pip install torch torchvision torchaudio \
  --extra-index-url https://download.pytorch.org/whl/cpu

pip install cv2
pip install -U ultralytics


Tello Setup
Hotspot: Tello-CBF1B9
Default IP: 192.168.10.1




<details>
<summary>No Longer Needed Steps</summary>
### Trying again with Windows Anaconda (Personal Computer Only)
- Download Anaconda & Git
- Setup Git Bash
conda init bash (Within Anavonda Navigator)
Reopen Bash

conda create -n python3.9 python=3.9
conda activate python3.9

conda install opencv numpy
pip install simple-pid djitellopy

</details>

</details>
