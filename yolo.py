import os
import getpass
import argparse
import threading
import socket
import time
import cv2
from ultralytics import YOLO

parser = argparse.ArgumentParser(
    description="Tello Drone YOLOv8 Person Tracking"
)
default_runtime = f"/tmp/runtime-{getpass.getuser()}"
parser.add_argument(
    "--xdg-runtime-dir",
    default=default_runtime,
    help="XDG runtime directory for GUI sockets"
)
args = parser.parse_args()

print("1) Ensuring DISPLAY for OpenCV GUI")
os.environ["DISPLAY"] = ":0"
print(f"   – DISPLAY set to {os.environ['DISPLAY']}")
os.makedirs(args.xdg_runtime_dir, exist_ok=True)
os.environ["XDG_RUNTIME_DIR"] = args.xdg_runtime_dir
print(f"2) XDG_RUNTIME_DIR set to {args.xdg_runtime_dir}")

print("3) Imported threading, socket, time, and OpenCV")

print("4) Imported Ultralytics YOLOv8 API")

TELLO_IP = "192.168.10.1"
TELLO_CMD_PORT = 8889
COMMAND_ADDRESS = (TELLO_IP, TELLO_CMD_PORT)
print(f"5) Tello command address: {COMMAND_ADDRESS}")

cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind(("", 9000))
print("6) Bound command socket on port 9000")

state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_sock.bind(("", 8890))
print("7) Bound state socket on port 8890")

def send_command(cmd: str):
    print(f" → Sending: {cmd}")
    cmd_sock.sendto(cmd.encode("utf-8"), COMMAND_ADDRESS)
    time.sleep(0.05) 

def state_receiver():
    print("8) State receiver thread started")
    while True:
        try:
            data, _ = state_sock.recvfrom(1024)
            print("   State:", data.decode().strip())
        except Exception as e:
            print("   State error:", e)
            break

_frame_lock = threading.Lock()
_latest_frame = None
_stop_event = threading.Event()

def capture_thread(cap):
    global _latest_frame
    print("10) Capture thread started")
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    while not _stop_event.is_set():
        cap.grab()
        ret, frame = cap.read()
        if not ret:
            continue
        with _frame_lock:
            _latest_frame = frame
    print("11) Capture thread exiting")

def inference_thread(model):
    print("12) Inference/display thread started")
    cv2.namedWindow("Tello YOLOv8", cv2.WINDOW_NORMAL)
    while not _stop_event.is_set():
        with _frame_lock:
            frame = _latest_frame.copy() if _latest_frame is not None else None
        if frame is None:
            continue

        results = model(frame)
        annotated = results[0].plot()

        cv2.imshow("Tello YOLOv8", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("    'q' pressed; exiting video")
            _stop_event.set()
            break

    cv2.destroyAllWindows()
    print("13) Inference/display thread exiting")

def video_thread():
    print("14) Video thread: loading YOLO model")
    model = YOLO("yolov8n.pt")

    url = "udp://@:11111"
    attempts = 0
    cap = None
    while True:
        print(f"15) Opening stream attempt {attempts+1}")
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            print("   Stream opened")
            break
        print("   Failed; retrying in 3 s")
        attempts += 1
        time.sleep(3)

    t_cap = threading.Thread(target=capture_thread, args=(cap,), daemon=True)
    t_inf = threading.Thread(target=inference_thread, args=(model,), daemon=True)
    t_cap.start()
    t_inf.start()

    while not _stop_event.is_set():
        time.sleep(0.1)

    cap.release()
    print("16) Video thread exiting")

# ---- 17) Main Routine ----
def main():
    threading.Thread(target=state_receiver, daemon=True).start()
    threading.Thread(target=video_thread, daemon=True).start()

    print("17) Initializing Tello SDK mode")
    send_command("command")
    time.sleep(1)

    print("18) Starting video stream")
    send_command("streamon")
    time.sleep(1)

    print("19) Takeoff (hover)")
    send_command("takeoff")
    time.sleep(5)

    print("20) Hovering; press 'q' in window or Ctrl+C to land")
    try:
        while not _stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("21) Keyboard interrupt; landing")
        _stop_event.set()

    send_command("land")
    cmd_sock.close()
    state_sock.close()
    print("22) Cleanup complete; exiting")

if __name__ == "__main__":
    main()
