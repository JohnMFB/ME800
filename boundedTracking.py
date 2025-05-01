#!/usr/bin/env python3
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

# 1) Argument parsing
parser = argparse.ArgumentParser(description="Tello Drone YOLOv8 Person Tracking with ArUco Boundary")
default_runtime = f"/tmp/runtime-{getpass.getuser()}"
parser.add_argument("--xdg-runtime-dir", default=default_runtime, help="XDG runtime directory for GUI sockets")
parser.add_argument("--marker-size", type=float, default=0.20, help="Side length of your printed 6×6 ArUco markers in meters")
parser.add_argument("--num-markers", type=int, default=4, help="Number of corner markers (IDs 1..N)")
parser.add_argument("--scan-yaw-speed", type=int, default=30, help="Yaw velocity (–100..100) during scanning")
parser.add_argument("--scan-timeout", type=float, default=20.0, help="Timeout (s) to complete marker/person scanning loops")
args = parser.parse_args()

# 2) Ensure DISPLAY and XDG_RUNTIME_DIR
os.environ["DISPLAY"] = ":0"
os.makedirs(args.xdg_runtime_dir, exist_ok=True)
os.environ["XDG_RUNTIME_DIR"] = args.xdg_runtime_dir

# 3) Networking Setup for Tello
TELLO_IP = "192.168.10.1"
TELLO_CMD_PORT = 8889
COMMAND_ADDRESS = (TELLO_IP, TELLO_CMD_PORT)
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind(("", 9000))

def send_command(cmd: str):
    print(f"→ CMD ▶ {cmd}")
    cmd_sock.sendto(cmd.encode("utf-8"), COMMAND_ADDRESS)
    time.sleep(0.05)

def send_rc(lr: int, fb: int, ud: int, yaw: int):
    send_command(f"rc {lr} {fb} {ud} {yaw}")

# 4) Shared state
_frame_lock = threading.Lock()
_latest_frame = None
_stop_event = threading.Event()
marker_positions = {}  # id → [X,Y,Z]
boundary = None       # (x_min, x_max, y_min, y_max)
pos_x = pos_y = 0.0

# 5) Load calibration & ArUco setup
calib = np.load("calibration.npz")
cam_mtx = calib["camera_matrix"]
dist_coefs = calib["dist_coefs"]
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
aruco_params = aruco.DetectorParameters()

# 6) Frame capture
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

def get_latest_frame():
    with _frame_lock:
        return None if _latest_frame is None else _latest_frame.copy()

# 7) Inference/tracking
def inference_thread(model):
    global pos_x, pos_y
    cv2.namedWindow("Tello YOLOv8", cv2.WINDOW_NORMAL)

    # PID gains & setpoints
    KP_Y, KI_Y, KD_Y = 150.0, 0.2, 30.0
    KP_F, KI_F, KD_F = 120.0, 0.2, 25.0
    KP_U, KI_U, KD_U = 100.0, 0.2, 25.0
    WIDTH_SET, TOP_SET = 0.60, 0.20

    prev_y = prev_f = prev_u = 0.0
    int_y = int_f = int_u = 0.0
    prev_t = time.time()

    while not _stop_event.is_set():
        frame = get_latest_frame()
        if frame is None:
            continue
        img = cv2.resize(frame, (640, 480))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # update pose via ArUco if available
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, args.marker_size, cam_mtx, dist_coefs)
            id0 = int(ids.flatten()[0]); idx = list(ids.flatten()).index(id0)
            if id0 in marker_positions:
                cam_t = tvecs[idx][0]
                pos_x = marker_positions[id0][0] - cam_t[0]
                pos_y = marker_positions[id0][1] - cam_t[1]

        # YOLO inference
        results = model(img, imgsz=480)
        annotated = results[0].plot()
        cv2.imshow("Tello YOLOv8", annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            _stop_event.set(); break

        boxes = results[0].boxes.xyxy.cpu().numpy()
        cls = results[0].boxes.cls.cpu().numpy().astype(int)
        persons = [b for b, c in zip(boxes, cls) if c == 0]
        if not persons:
            send_rc(0, 0, 0, 0); continue

        areas = [(b[2]-b[0])*(b[3]-b[1]) for b in persons]
        x1, y1, x2, y2 = persons[int(np.argmax(areas))]

        t = time.time(); dt = t - prev_t if t > prev_t else 1e-6
        cy = (x1 + x2)/2/640 - 0.5; int_y += cy*dt; der_y = (cy-prev_y)/dt
        yaw = int(np.clip(KP_Y*cy + KI_Y*int_y + KD_Y*der_y, -100,100))
        ef = WIDTH_SET - ((x2 - x1)/640); int_f += ef*dt; der_f = (ef-prev_f)/dt
        fb = int(np.clip(KP_F*ef + KI_F*int_f + KD_F*der_f, -100,100))
        eu = TOP_SET - (y1/480); int_u += eu*dt; der_u = (eu-prev_u)/dt
        ud = int(np.clip(KP_U*eu + KI_U*int_u + KD_U*der_u, -100,100))

        x_min, x_max, y_min, y_max = boundary
        pred_y = pos_y + fb/100.0*dt
        if pred_y<y_min or pred_y>y_max: fb=0

        send_rc(0, fb, ud, yaw)
        prev_y, prev_f, prev_u, prev_t = cy, ef, eu, t

    cv2.destroyAllWindows()

# 8) Video + scanning
def video_thread():
    model = YOLO("yolov8n.pt")
    url = "udp://@:11111"
    print("[VIDEO] waiting for stream directory...")
    while not os.path.exists(args.xdg_runtime_dir) and not _stop_event.is_set(): time.sleep(0.1)
    print("[VIDEO] opening video stream")
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if not cap.isOpened(): print("[VIDEO] FAILED to open stream"); _stop_event.set(); return
    print("[VIDEO] stream opened")
    threading.Thread(target=capture_thread, args=(cap,), daemon=True).start()

    print(f"[SCAN] Rotating to detect {args.num_markers} markers")
    send_rc(0,0,0,args.scan_yaw_speed)
    start=time.time()
    while len(marker_positions)<args.num_markers:
        if time.time()-start>args.scan_timeout: break
        frame=get_latest_frame()
        if frame is None: continue
        gray=cv2.cvtColor(cv2.resize(frame,(640,480)),cv2.COLOR_BGR2GRAY)
        corners,ids,_=aruco.detectMarkers(gray,aruco_dict,parameters=aruco_params)
        if ids is not None:
            rvecs,tvecs,_=aruco.estimatePoseSingleMarkers(corners,args.marker_size,cam_mtx,dist_coefs)
            for i,idv in enumerate(ids.flatten()):
                if idv not in marker_positions: marker_positions[idv]=tvecs[i][0]; print(f"[SCAN] found marker {idv}")
    send_rc(0,0,0,0)
    if marker_positions: print("[SCAN] Marker scan complete")

    xs=[v[0] for v in marker_positions.values()]; ys=[v[1] for v in marker_positions.values()]
    m=0.3048; global boundary; boundary=(min(xs)+m,max(xs)-m,min(ys)+m,max(ys)-m)
    print(f"[BOUNDARY] x:[{boundary[0]:.2f},{boundary[1]:.2f}], y:[{boundary[2]:.2f},{boundary[3]:.2f}]")

    print("[SCAN] Rotating to locate person")
    send_rc(0,0,0,args.scan_yaw_speed)
    start=time.time(); found=False
    while not found and time.time()-start<args.scan_timeout:
        frame=get_latest_frame();
        if frame is None: continue
        res=model(cv2.resize(frame,(640,480)),imgsz=480)
        if any(c==0 for c in res[0].boxes.cls.cpu().numpy().astype(int)): found=True
    send_rc(0,0,0,0)
    print("[SCAN] Person scan complete")

    threading.Thread(target=inference_thread,args=(model,),daemon=True).start()

# 9) Main routine
def main():
    send_command("command"); time.sleep(1)
    send_command("streamon"); time.sleep(2)
    send_command("takeoff"); time.sleep(5)
    threading.Thread(target=video_thread,daemon=True).start()
    try:
        while not _stop_event.is_set(): time.sleep(0.5)
    except KeyboardInterrupt:
        _stop_event.set()
    send_command("land"); cmd_sock.close()

if __name__=="__main__":
    main()
