import os
import getpass
import argparse
import threading
import socket
import time
import cv2
import numpy as np
from ultralytics import YOLO

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

os.environ["DISPLAY"] = ":0"
os.makedirs(args.xdg_runtime_dir, exist_ok=True)
os.environ["XDG_RUNTIME_DIR"] = args.xdg_runtime_dir

TELLO_IP = "192.168.10.1"
TELLO_CMD_PORT = 8889
COMMAND_ADDRESS = (TELLO_IP, TELLO_CMD_PORT)

cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind(("", 9000))

state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_sock.bind(("", 8890))

def send_command(cmd: str):
    print(f"-> CMD -> {cmd}")
    cmd_sock.sendto(cmd.encode("utf-8"), COMMAND_ADDRESS)
    time.sleep(0.05)

def send_rc(lr: int, fb: int, ud: int, yaw: int):

    cmd = f"rc {lr} {fb} {ud} {yaw}"
    send_command(cmd)

def state_receiver():
    while True:
        try:
            data, _ = state_sock.recvfrom(1024)
            print("⟳ STATE:", data.decode().strip())
        except:
            break

_frame_lock = threading.Lock()
_latest_frame = None
_stop_event   = threading.Event()

def capture_thread(cap):
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
    print("[INFER ] started")
    cv2.namedWindow("Tello YOLOv8", cv2.WINDOW_NORMAL)

    DESIRED_WIDTH_RATIO = 0.60   
    DESIRED_TOP_RATIO   = 0.20  

    KP_YAW, KI_YAW, KD_YAW = 150.0, 0.2, 30.0
    KP_FB,  KI_FB,  KD_FB  = 120.0, 0.2, 25.0
    KP_UD,  KI_UD,  KD_UD  = 100.0, 0.2, 25.0

    prev_err_yaw = prev_err_fb = prev_err_ud = 0.0
    int_err_yaw  = int_err_fb  = int_err_ud  = 0.0
    prev_time    = time.time()

    while not _stop_event.is_set():
        with _frame_lock:
            frame = _latest_frame.copy() if _latest_frame is not None else None
        if frame is None:
            continue

        resized = cv2.resize(frame, (640, 480))
        results = model(resized, imgsz=480)
        annotated = results[0].plot()
        cv2.imshow("Tello YOLOv8", annotated)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            _stop_event.set()
            break

        # Person detection
        boxes   = results[0].boxes.xyxy.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy().astype(int)
        persons = [b for b, c in zip(boxes, classes) if c == 0]

        if not persons:
            send_rc(0, 0, 0, 0)
            continue

        # Largest person
        areas = [(b[2]-b[0])*(b[3]-b[1]) for b in persons]
        x1, y1, x2, y2 = persons[int(np.argmax(areas))]

        H, W = resized.shape[:2]
        box_w = x2 - x1

        # Compute time delta
        curr_time = time.time()
        dt = curr_time - prev_time if curr_time > prev_time else 1e-6

        cx = (x1 + x2) / 2
        err_yaw = (cx / W) - 0.5
        int_err_yaw += err_yaw * dt
        der_err_yaw = (err_yaw - prev_err_yaw) / dt
        yaw_output = (KP_YAW * err_yaw +
                      KI_YAW * int_err_yaw +
                      KD_YAW * der_err_yaw)
        yaw = int(np.clip(yaw_output, -100, 100))

        curr_ratio = box_w / W
        err_fb = DESIRED_WIDTH_RATIO - curr_ratio
        int_err_fb += err_fb * dt
        der_err_fb = (err_fb - prev_err_fb) / dt
        fb_output = (KP_FB * err_fb +
                     KI_FB * int_err_fb +
                     KD_FB * der_err_fb)
        fb = int(np.clip(fb_output, -100, 100))

        err_ud = DESIRED_TOP_RATIO - (y1 / H)
        int_err_ud += err_ud * dt
        der_err_ud = (err_ud - prev_err_ud) / dt
        ud_output = (KP_UD * err_ud +
                     KI_UD * int_err_ud +
                     KD_UD * der_err_ud)
        ud = int(np.clip(ud_output, -100, 100))

        send_rc(0, fb, ud, yaw)

        prev_err_yaw = err_yaw
        prev_err_fb  = err_fb
        prev_err_ud  = err_ud
        prev_time    = curr_time

    cv2.destroyAllWindows()
    print("[INFER ] exiting")

def video_thread():
    print("[VIDEO ] loading model and opening stream")
    model = YOLO("yolov8n.pt")
    url   = "udp://@:11111"

    print("[VIDEO ] waiting for streamon")
    while not _stop_event.is_set() and not os.path.exists(args.xdg_runtime_dir):
        time.sleep(0.1)

    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[VIDEO ] !FAILED to open stream")
        _stop_event.set()
        return
    print("[VIDEO ] stream opened")

    threading.Thread(target=capture_thread,   args=(cap,),   daemon=True).start()
    threading.Thread(target=inference_thread, args=(model,), daemon=True).start()

    while not _stop_event.is_set():
        time.sleep(0.1)
    print("[VIDEO ] exiting")

def main():
    print("[MAIN ] starting state receiver")
    threading.Thread(target=state_receiver, daemon=True).start()

    send_command("command");  time.sleep(1)
    send_command("streamon"); time.sleep(1)
    print("[MAIN ] streamon complete, launching video threads")
    threading.Thread(target=video_thread, daemon=True).start()

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
