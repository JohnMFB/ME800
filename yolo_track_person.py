#!/usr/bin/env python3
import os
import getpass
import argparse
import threading
import socket
import time
import cv2
import numpy as np
from ultralytics import YOLO

# ---- 1) Argument Parsing (including XDG_RUNTIME_DIR) ----
parser = argparse.ArgumentParser(
    description="Tello Drone YOLOv8 Person Tracking with PID-friendly rc control"
)
default_runtime = f"/tmp/runtime-{getpass.getuser()}"
parser.add_argument(
    "--xdg-runtime-dir",
    default=default_runtime,
    help="XDG runtime directory for GUI sockets"
)
args = parser.parse_args()

# ---- 2) Ensure DISPLAY and XDG_RUNTIME_DIR for OpenCV GUI ----
os.environ["DISPLAY"] = ":0"
os.makedirs(args.xdg_runtime_dir, exist_ok=True)
os.environ["XDG_RUNTIME_DIR"] = args.xdg_runtime_dir

# ---- 3) Networking Setup for Tello ----
TELLO_IP = "192.168.10.1"
TELLO_CMD_PORT = 8889
COMMAND_ADDRESS = (TELLO_IP, TELLO_CMD_PORT)

cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind(("", 9000))

state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_sock.bind(("", 8890))

def send_command(cmd: str):
    """Send a single command to the Tello."""
    cmd_sock.sendto(cmd.encode("utf-8"), COMMAND_ADDRESS)
    time.sleep(0.05)

def send_rc(lr: int, fb: int, ud: int, yaw: int):
    """
    Send continuous velocity command.
    lr: left/right  (-100..100)
    fb: forward/back(-100..100)
    ud: up/down    (-100..100)
    yaw: yaw       (-100..100)
    """
    cmd = f"rc {lr} {fb} {ud} {yaw}"
    send_command(cmd)

def state_receiver():
    """Continuously receive and print Tello state messages."""
    while True:
        try:
            data, _ = state_sock.recvfrom(1024)
            print("State:", data.decode().strip())
        except:
            break

# ---- 4) Video Capture + Inference Threads ----
_frame_lock = threading.Lock()
_latest_frame = None
_stop_event = threading.Event()

def capture_thread(cap):
    """Grab only the latest frame to minimize latency."""
    global _latest_frame
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    while not _stop_event.is_set():
        cap.grab()
        ret, frame = cap.read()
        if not ret:
            continue
        with _frame_lock:
            _latest_frame = frame
    cap.release()

def inference_thread(model):
    """Run YOLO, compute control signals, and display."""
    cv2.namedWindow("Tello YOLOv8", cv2.WINDOW_NORMAL)

    # Desired set‑points (fractions of frame)
    DESIRED_WIDTH_RATIO  = 0.3    # want person to occupy ~30% of frame width
    DESIRED_CENTER_Y     = 0.33   # want bbox center at ~1/3 down (top 2/3)
    # Proportional gains (tune these!)
    KP_YAW   = 80   # yaw speed per unit horizontal error
    KP_FB    = 80   # forward/back speed per unit width error
    KP_UD    = 80   # up/down   speed per unit vertical error

    while not _stop_event.is_set():
        frame = None
        with _frame_lock:
            if _latest_frame is not None:
                frame = _latest_frame.copy()
        if frame is None:
            continue

        h, w = frame.shape[:2]
        resized = cv2.resize(frame, (640, 480))
        results = model(resized, imgsz=480)
        annotated = results[0].plot()
        cv2.imshow("Tello YOLOv8", annotated)

        # quit on 'q'
        if cv2.waitKey(1) & 0xFF == ord("q"):
            _stop_event.set()
            break

        # extract person detections
        det = results[0]
        boxes   = det.boxes.xyxy.cpu().numpy()
        classes = det.boxes.cls.cpu().numpy().astype(int)
        person_boxes = [b for b, c in zip(boxes, classes) if c == 0]
        if not person_boxes:
            # no person → stop movement
            send_rc(0, 0, 0, 0)
            continue

        # pick the largest person (by box area)
        areas = [(b[2]-b[0])*(b[3]-b[1]) for b in person_boxes]
        idx = int(np.argmax(areas))
        x1, y1, x2, y2 = person_boxes[idx]

        # compute normalized errors
        box_width       = x2 - x1
        center_x        = (x1 + x2) / 2
        center_y        = (y1 + y2) / 2

        err_horiz       = (center_x / resized.shape[1]) - 0.5
        err_dist        = DESIRED_WIDTH_RATIO - (box_width / resized.shape[1])
        err_vert        = DESIRED_CENTER_Y - (center_y / resized.shape[0])

        # P‑control outputs
        yaw_speed = int(np.clip(KP_YAW * err_horiz, -100, 100))
        fb_speed  = int(np.clip(KP_FB  * err_dist,  -100, 100))
        ud_speed  = int(np.clip(KP_UD  * err_vert,  -100, 100))

        # send single rc command per frame
        send_rc(lr=0, fb=fb_speed, ud=ud_speed, yaw=yaw_speed)

    cv2.destroyAllWindows()

def video_thread():
    model = YOLO("yolov8n.pt")
    url = "udp://@:11111"
    # open stream
    cap = None
    while True:
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            break
        time.sleep(1)
    # start threads
    threading.Thread(target=capture_thread, args=(cap,), daemon=True).start()
    threading.Thread(target=inference_thread, args=(model,), daemon=True).start()
    # wait until stop
    while not _stop_event.is_set():
        time.sleep(0.1)

# ---- 5) Main Routine ----
def main():
    threading.Thread(target=state_receiver, daemon=True).start()
    threading.Thread(target=video_thread, daemon=True).start()

    # init & takeoff
    send_command("command");  time.sleep(1)
    send_command("streamon"); time.sleep(1)
    send_command("takeoff");  time.sleep(5)

    try:
        while not _stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        _stop_event.set()

    send_command("land")
    cmd_sock.close()
    state_sock.close()

if __name__ == "__main__":
    main()
