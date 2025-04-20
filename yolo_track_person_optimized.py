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
    print(f"→ CMD ▶ {cmd}")
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
            print("⟳ STATE:", data.decode().strip())
        except:
            break

# ---- 4) Video Capture + Inference Threads ----
_frame_lock = threading.Lock()
_latest_frame = None
_stop_event   = threading.Event()

def capture_thread(cap):
    """Grab only the latest frame to minimize latency."""
    global _latest_frame
    print("[CAPTURE] started")
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    while not _stop_event.is_set():
        cap.grab()
        ret, frame = cap.read()
        if not ret:
            continue
        with _frame_lock:
            _latest_frame = frame
    cap.release()
    print("[CAPTURE] exiting")

def inference_thread(model):
    """Run YOLO, compute control signals, and display."""
    print("[INFER ] started")
    cv2.namedWindow("Tello YOLOv8", cv2.WINDOW_NORMAL)

    # TUNABLE SETPOINTS & GAINS
    DESIRED_WIDTH_RATIO = 0.60   # want person to fill 60% of frame width
    DESIRED_TOP_RATIO   = 0.20   # want top of bbox at 20% down from top
    KP_YAW   = 80
    KP_FB    = 80
    KP_UD    = 80

    while not _stop_event.is_set():
        with _frame_lock:
            frame = _latest_frame.copy() if _latest_frame is not None else None
        if frame is None:
            continue

        # inference
        resized = cv2.resize(frame, (640, 480))
        results = model(resized, imgsz=480)
        annotated = results[0].plot()
        cv2.imshow("Tello YOLOv8", annotated)

        # quit on 'q'
        if cv2.waitKey(1) & 0xFF == ord("q"):
            _stop_event.set()
            break

        # find the largest person
        boxes   = results[0].boxes.xyxy.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy().astype(int)
        persons = [b for b, c in zip(boxes, classes) if c == 0]
        if not persons:
            send_rc(0, 0, 0, 0)
            continue

        # pick the biggest box by area
        areas = [(b[2]-b[0])*(b[3]-b[1]) for b in persons]
        x1, y1, x2, y2 = persons[int(np.argmax(areas))]

        H, W = resized.shape[:2]
        box_w = x2 - x1

        # ---- horizontal (yaw)
        cx = (x1 + x2) / 2
        err_h = (cx / W) - 0.5
        yaw = int(np.clip(KP_YAW * err_h, -100, 100))

        # ---- distance (forward/back)
        curr_ratio = box_w / W
        err_d = DESIRED_WIDTH_RATIO - curr_ratio
        fb = int(np.clip(KP_FB * err_d, -100, 100))

        # ---- vertical (up/down) using top of bbox
        err_v = DESIRED_TOP_RATIO - (y1 / H)
        ud = int(np.clip(KP_UD * err_v, -100, 100))

        send_rc(0, fb, ud, yaw)

    cv2.destroyAllWindows()
    print("[INFER ] exiting")

def video_thread():
    print("[VIDEO ] loading model and opening stream")
    model = YOLO("yolov8n.pt")
    url   = "udp://@:11111"

    # wait until stream actually turns on
    print("[VIDEO ] waiting for streamon")
    while _stop_event.is_set() is False and not os.path.exists(args.xdg_runtime_dir):
        time.sleep(0.1)
    # now open the UDP port
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[VIDEO ] !FAILED to open stream")
        _stop_event.set()
        return
    print("[VIDEO ] stream opened")

    # start sub‑threads
    threading.Thread(target=capture_thread,   args=(cap,),     daemon=True).start()
    threading.Thread(target=inference_thread, args=(model,),   daemon=True).start()

    # just block until user quits
    while not _stop_event.is_set():
        time.sleep(0.1)
    print("[VIDEO ] exiting")

# ---- 5) Main Routine ----
def main():
    print("[MAIN ] starting state receiver")
    threading.Thread(target=state_receiver, daemon=True).start()

    # initialize SDK
    send_command("command");  time.sleep(1)

    # turn on the stream *before* starting video_thread
    send_command("streamon"); time.sleep(1)
    print("[MAIN ] streamon complete, launching video threads")
    threading.Thread(target=video_thread, daemon=True).start()

    # take off once video is live
    send_command("takeoff");  time.sleep(5)
    print("[MAIN ] takeoff complete — tracking now")

    try:
        while not _stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("[MAIN ] Ctrl+C detected, landing...")
        _stop_event.set()

    send_command("land")
    cmd_sock.close()
    state_sock.close()
    print("[MAIN ] cleanup done, exiting")

if __name__ == "__main__":
    main()
