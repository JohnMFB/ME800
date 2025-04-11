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


sudo apt update
sudo apt upgrade

Github Env:
python3 -m venv Tello
Activating Tello:
source Tello/bin/activate

Python Packages Within Shared env:
cv2

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

