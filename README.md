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

* For aruco, opencv-contrib-python needed
pip uninstall opencv-python
pip install opencv-contrib-python


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

