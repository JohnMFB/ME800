#!/usr/bin/env python3
import os
import getpass
import argparse
import threading
import socket
import time
import cv2
import numpy as np

# ---- 1) Argument Parsing (including XDG_RUNTIME_DIR) ----
parser = argparse.ArgumentParser(
    description="Tello Drone Camera Calibration (Chessboard Marker)"
)
default_runtime = f"/tmp/runtime-{getpass.getuser()}"
parser.add_argument(
    "--xdg-runtime-dir",
    default=default_runtime,
    help="XDG runtime directory for GUI sockets"
)
parser.add_argument(
    "--num-frames", type=int, default=20,
    help="Number of calibration frames to capture"
)
# Default to 9x6 if using the OpenCV doc pattern.png (9 cols, 6 rows)
parser.add_argument(
    "--pattern-size", type=int, nargs=2, default=[9,6],
    help="Chessboard pattern inner corners: columns rows (e.g. 9 6)"
)
parser.add_argument(
    "--square-size", type=float, default=0.03,
    help="Size of one square in meters"
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

def send_command(cmd: str):
    print(f"→ CMD ▶ {cmd}")
    cmd_sock.sendto(cmd.encode("utf-8"), COMMAND_ADDRESS)
    time.sleep(0.05)

# ---- 4) Shared State ----
_frame_lock   = threading.Lock()
_latest_frame = None
_stop_event   = threading.Event()

# ---- 5) Frame Capture Thread ----
def capture_thread():
    global _latest_frame
    # give the drone time to start streaming
    time.sleep(2.0)
    cap = cv2.VideoCapture("udp://@:11111", cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print("[CAPTURE] Failed to open video stream. Ensure Tello is streaming.")
        _stop_event.set()
        return
    print("[CAPTURE] Video stream opened")
    while not _stop_event.is_set():
        cap.grab()
        ret, frame = cap.read()
        if not ret:
            continue
        with _frame_lock:
            _latest_frame = frame
    cap.release()
    print("[CAPTURE] Exiting")

# ---- 6) Calibration Loop ----
def main():
    # Enter SDK mode and start video
    send_command("command")
    time.sleep(1)
    send_command("streamon")
    time.sleep(2)

    # Prepare object points for chessboard
    cols, rows = args.pattern_size
    objp = np.zeros((rows*cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= args.square_size

    objpoints = []
    imgpoints = []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
    print(f"Press 'c' when the checkerboard is fully visible; need {args.num_frames} captures.")

    while not _stop_event.is_set() and len(objpoints) < args.num_frames:
        frame = None
        with _frame_lock:
            if _latest_frame is not None:
                frame = _latest_frame.copy()
        if frame is None:
            time.sleep(0.01)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray,
            (cols, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        vis = frame.copy()
        if ret:
            cv2.drawChessboardCorners(vis, (cols, rows), corners, ret)
        cv2.imshow("Calibration", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('c') and ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)
            print(f"Captured {len(objpoints)}/{args.num_frames}")
        elif key == ord('q'):
            _stop_event.set()
            break

    cv2.destroyAllWindows()
    send_command("land")

    if len(objpoints) >= args.num_frames:
        print("Calibrating...")
        ret, mtx, dist, _, _ = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )
        print("RMS error:", ret)
        print("Camera matrix:\n", mtx)
        print("Distortion coefficients:", dist.ravel())
        np.savez("calibration.npz", camera_matrix=mtx, dist_coefs=dist)
        print("Saved calibration.npz")
    else:
        print(f"Only {len(objpoints)} frames captured; calibration aborted.")

if __name__ == "__main__":
    # Start video capture
    threading.Thread(target=capture_thread, daemon=True).start()
    try:
        main()
    except KeyboardInterrupt:
        _stop_event.set()
