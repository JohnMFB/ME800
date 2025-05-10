import os
import getpass
import argparse
import threading
import socket
import time

import cv2
import cv2.aruco as aruco
import numpy as np
from ultralytics import YOLO
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

parser = argparse.ArgumentParser(
    description="Tello Drone YOLOv8 Person Tracking with ArUco Boundary")
default_runtime = f"/tmp/runtime-{getpass.getuser()}"
parser.add_argument("--xdg-runtime-dir", default=default_runtime,
                    help="XDG runtime directory for GUI sockets")
parser.add_argument("--marker-size", type=float, default=0.20,
                    help="Side length of your printed ArUco markers in meters")
parser.add_argument("--num-markers", type=int, default=4,
                    help="Number of corner markers (IDs 1..N)")
parser.add_argument("--scan-yaw-speed", type=int, default=30,
                    help="Continuous yaw speed during scanning (–100..100)")
parser.add_argument("--scan-timeout", type=float, default=30.0,
                    help="Timeout (s) for each marker scan")
parser.add_argument("--center-thresh", type=int, default=30,
                    help="Pixel tolerance for ‘centered’ detection")
parser.add_argument("--stability-frames", type=int, default=10,
                    help="Consecutive frames to average once centered")
parser.add_argument("--pid-delay", type=float, default=0.05,
                    help="Delay (s) between PID loop iterations")
args = parser.parse_args()

os.environ["DISPLAY"] = ":0"
os.makedirs(args.xdg_runtime_dir, exist_ok=True)
os.environ["XDG_RUNTIME_DIR"] = args.xdg_runtime_dir

TELLO_IP, TELLO_CMD_PORT = "192.168.10.1", 8889
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind(("", 9000))

def send_command(cmd: str):
    """Send a text command to Tello via UDP."""
    print(f"→ CMD ▶ {cmd}")
    cmd_sock.sendto(cmd.encode(), (TELLO_IP, TELLO_CMD_PORT))
    time.sleep(0.1)

def send_rc(lr, fb, ud, yaw):
    """Send rc control (keeps drone alive too)."""
    send_command(f"rc {lr} {fb} {ud} {yaw}")

_stop_event = threading.Event()
def keepalive():
    while not _stop_event.is_set():
        send_command("command")
        time.sleep(5)

calib = np.load("calibration.npz")  
cam_mtx, dist_coefs = calib["camera_matrix"], calib["dist_coefs"]
aruco_dict   = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
aruco_params = aruco.DetectorParameters()
aruco_params.adaptiveThreshWinSizeMax = 50   
aruco_params.errorCorrectionRate     = 0.8

_frame_lock   = threading.Lock()
_latest_frame = None

def capture_thread(cap):
    global _latest_frame
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    while not _stop_event.is_set():
        cap.grab()
        ret, frame = cap.read()
        if ret:
            with _frame_lock:
                _latest_frame = frame
    cap.release()

def get_frame():
    with _frame_lock:
        return None if _latest_frame is None else _latest_frame.copy()


def scan_marker(mid):

    IMG_W, IMG_H = 640, 480
    start = time.time()
    send_rc(0, 0, 0, args.scan_yaw_speed) 
    stable = []

    while time.time() - start < args.scan_timeout:
        frame = get_frame()
        if frame is None:
            time.sleep(0.02)
            continue

        cv2.imshow("Tello Markers", frame)  
        cv2.waitKey(1)                      

        gray = cv2.cvtColor(cv2.resize(frame,(IMG_W,IMG_H)), cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if ids is not None and mid in ids.flatten():
            idx = list(ids.flatten()).index(mid)
            cx = corners[idx][0][:,0].mean()
            if abs(cx - IMG_W/2) <= args.center_thresh:
                _, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[idx:idx+1], args.marker_size, cam_mtx, dist_coefs)
                stable.append(tvecs[0][0])
                if len(stable) >= args.stability_frames:
                    break
        time.sleep(0.02)

    send_rc(0, 0, 0, 0)
    if not stable:
        raise RuntimeError(f"Marker {mid} scan failed")
    avg = np.mean(stable, axis=0)
    return float(avg[0]), float(avg[2])

def save_boundary_plots(boundary,
                        png_path="boundary.png",
                        pdf_path="boundary.pdf"):
    x_min, x_max, z_min, z_max = boundary

    scale, m = 200, 50
    w = int((x_max - x_min)*scale) + 2*m
    h = int((z_max - z_min)*scale) + 2*m
    img = np.full((h, w, 3), 255, dtype=np.uint8)
    pts = np.array([
        (int((x_min-x_min)*scale)+m, int((z_max-z_min)*scale)+m),
        (int((x_min-x_min)*scale)+m, int((z_min-z_min)*scale)+m),
        (int((x_max-x_min)*scale)+m, int((z_min-z_min)*scale)+m),
        (int((x_max-x_min)*scale)+m, int((z_max-z_min)*scale)+m),
    ])
    cv2.polylines(img, [pts], True, (0,0,0), 2)
    cv2.imwrite(png_path, img)

    fig, ax = plt.subplots()
    corners = [(x_min,z_max),(x_min,z_min),(x_max,z_min),(x_max,z_max)]
    poly = plt.Polygon(corners, closed=True, fill=None, edgecolor='black')
    ax.add_patch(poly)
    ax.set_xlim(x_min-0.5, x_max+0.5)
    ax.set_ylim(z_min-0.5, z_max+0.5)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    with PdfPages(pdf_path) as pdf:
        pdf.savefig(fig)
    plt.close(fig)

def video_thread():
    send_command("command")
    send_command("streamon")
    time.sleep(2)  

    cap = cv2.VideoCapture("udp://@:11111", apiPreference=cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[ERROR] Video stream failed")
        _stop_event.set()
        return


    send_command("takeoff")
    time.sleep(3) 

    threading.Thread(target=capture_thread, args=(cap,), daemon=True).start()
    cv2.namedWindow("Tello Markers", cv2.WINDOW_NORMAL)

    positions = {}
    for mid in range(1, args.num_markers+1):
        x_m, z_m = scan_marker(mid)
        positions[mid] = (x_m, z_m)
        print(f"[LOCK] Marker#{mid}: x={x_m:.2f}, z={z_m:.2f}")


    xs = [p[0] for p in positions.values()]
    zs = [p[1] for p in positions.values()]
    inset = 0.3048
    x_min, x_max = min(xs)-inset, max(xs)+inset
    z_min, z_max = min(zs)-inset, max(zs)+inset
    boundary = (x_min, x_max, z_min, z_max)
    print(f"[BOUNDARY] x[{x_min:.2f},{x_max:.2f}] z[{z_min:.2f},{z_max:.2f}]")


    save_boundary_plots(boundary)


    corners = [(x_min,z_min),(x_min,z_max),(x_max,z_max),(x_max,z_min)]
    for xm, zm in corners:
        send_command(f"go {int(xm*100)} {int(zm*100)} 0 20")
        time.sleep(5)


    cx = int(((x_min + x_max)/2)*100)
    cz = int(((z_min + z_max)/2)*100)
    send_command(f"go {cx} {cz} 0 20")
    time.sleep(5)


    threading.Thread(target=inference_thread,
                     args=(YOLO("yolov8n.pt"),), daemon=True).start()


def inference_thread(model):
    cv2.namedWindow("Tello YOLOv8", cv2.WINDOW_NORMAL)

if __name__ == "__main__":
    threading.Thread(target=keepalive, daemon=True).start()
    threading.Thread(target=video_thread, daemon=True).start()
    try:
        while not _stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        _stop_event.set()

    send_command("land")
    cmd_sock.close()
