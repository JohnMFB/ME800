# ME800
Tello Drone PID/L1 Adaptive Controller Project

#### Setup
- WSL, SSH, VSCode, Git Clone, ...
- Python
Creating

WSL Dependencies (Install in each device):
sudo apt install python3 python3-pip python3-venv -y

Github Env:
python3 -m venv env

Activating Env:
source env/bin/activate

Python Packages Within Shared env:


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