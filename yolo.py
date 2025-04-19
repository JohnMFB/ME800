import os
print("1) Ensuring DISPLAY for OpenCV GUI")
if "DISPLAY" not in os.environ:
    print("   – Setting DISPLAY to :0")
    os.environ["DISPLAY"] = ":0"

import threading, socket, time, cv2
print("2) Imported threading, socket, time, and OpenCV")

from ultralytics import YOLO
print("3) Imported Ultralytics YOLOv8 API")

# ---- Tello UDP Setup ----
TELLO_IP = '192.168.10.1'
TELLO_CMD_PORT = 8889
COMMAND_ADDRESS = (TELLO_IP, TELLO_CMD_PORT)
print(f"4) Tello command address: {COMMAND_ADDRESS}")

cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind(('', 9000))
print("5) Bound command socket on port 9000")

state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_sock.bind(('', 8890))
print("6) Bound state socket on port 8890")

def send_command(cmd):
    print(f" → Sending: {cmd}")
    cmd_sock.sendto(cmd.encode(), COMMAND_ADDRESS)
    time.sleep(0.05)

def state_receiver():
    print("7) State receiver thread started")
    while True:
        try:
            data, _ = state_sock.recvfrom(1024)
            print("   State:", data.decode().strip())
        except Exception as e:
            print("   State error:", e)
            break

# ---- Video + Detection Thread ----
def video_thread():
    print("8) Video thread: loading YOLO model")
    model = YOLO("yolov8n.pt")  # swap .n to .s/.m/.l/.x to change size

    cap = None
    attempts = 0
    url = "udp://@:11111"
    while cap is None or not cap.isOpened():
        print(f"9) Opening stream attempt {attempts+1}")
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            print("   Failed; retrying in 3 s")
            attempts += 1
            time.sleep(3)
        else:
            print("   Stream opened")

    print("10) Entering capture → detection loop")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        print("    Frame grabbed; running inference")
        results = model(frame)               # one‑line inference
        annotated = results[0].plot()        # draws boxes & labels

        cv2.imshow("Tello YOLOv8", annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("    'q' pressed; exiting")
            break
        time.sleep(1/30)                      # cap to ~30 FPS

    cap.release()
    cv2.destroyAllWindows()
    print("11) Video thread exiting")

# ---- Main Routine ----
def main():
    threading.Thread(target=state_receiver, daemon=True).start()
    threading.Thread(target=video_thread, daemon=True).start()

    print("12) Initializing Tello SDK mode")
    send_command("command")
    time.sleep(1)

    print("13) Starting video stream")
    send_command("streamon")
    time.sleep(1)

    print("14) Takeoff (hover)")
    send_command("takeoff")
    time.sleep(5)

    print("15) Hovering; press 'q' in window or Ctrl+C to land")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("16) Keyboard interrupt; landing")

    send_command("land")
    cmd_sock.close()
    state_sock.close()
    print("17) Cleanup complete")

if __name__ == "__main__":
    main()
