import os
import threading
import socket
import time
import math
import cv2

# Ensure the DISPLAY is set for OpenCV windows.
# For WSL1, this should work since the Windows X server (e.g., VcXsrv) is running.
if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"

# ---------------------------
# Tello SDK Command Setup
# ---------------------------
TELLO_IP = '192.168.10.1'
TELLO_CMD_PORT = 8889
COMMAND_ADDRESS = (TELLO_IP, TELLO_CMD_PORT)

# Create UDP socket for sending commands.
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind(('', 9000))

# Create UDP socket for receiving state.
state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_sock.bind(('', 8890))

def send_command(command):
    """Send a command to Tello and print it."""
    print("Sending:", command)
    cmd_sock.sendto(command.encode('utf-8'), COMMAND_ADDRESS)
    time.sleep(0.05)

def state_receiver():
    """
    Continuously receive state data.
    (This example does not parse state into a global variable, but you can expand it later.)
    """
    while True:
        try:
            data, _ = state_sock.recvfrom(1024)
            state_str = data.decode('utf-8').strip()
            # Future improvements: parse state_str and store needed values.
        except Exception as e:
            print("State receiver error:", e)
            break

# ---------------------------
# Video Thread with Retry
# ---------------------------
def video_thread():
    """
    Capture and display the Tello video stream using FFmpeg.
    The Tello sends video via UDP on port 11111.
    This function will retry until it obtains a working stream.
    Notice that the URL now is "udp://@:11111" to match ffplay's working syntax.
    """
    cap = None
    attempts = 0
    # The working URL based on your ffplay test.
    stream_url = "udp://@:11111"
    while cap is None or not cap.isOpened():
        print("Attempting to open video stream (attempt {})...".format(attempts + 1))
        cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            print("Error: Could not open video stream. Make sure you sent 'streamon', your X server is running, and your network is bridged correctly.")
            attempts += 1
            time.sleep(3)  # Wait a few seconds before retrying.
        else:
            print("Video stream opened successfully.")
    while True:
        ret, frame = cap.read()
        if ret:
            cv2.imshow("Tello Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

# ---------------------------
# Open-Loop Control Functions
# ---------------------------
def rotate_by(angle, yaw_rate=30, dt=0.1):
    """
    Rotate the drone by 'angle' degrees.
    Positive angle rotates clockwise.
    The motion is broken into small steps with dt control intervals.
    """
    t_total = abs(angle) / yaw_rate
    iterations = int(t_total / dt)
    print("Rotating by {}° (expected time: {:.2f} sec, {} iterations)".format(angle, t_total, iterations))
    for _ in range(iterations):
        if angle > 0:
            send_command("rc 0 0 0 {}".format(yaw_rate))
        else:
            send_command("rc 0 0 0 {}".format(-yaw_rate))
        time.sleep(dt)
    send_command("rc 0 0 0 0")

def move_forward(distance, forward_speed=30, dt=0.1):
    """
    Move forward a specified distance (cm) in small increments.
    forward_speed is in cm/s.
    """
    t_total = distance / forward_speed
    iterations = int(t_total / dt)
    print("Moving forward {:.2f} cm (expected time: {:.2f} sec, {} iterations)".format(distance, t_total, iterations))
    for _ in range(iterations):
        send_command("rc 0 {} 0 0".format(forward_speed))
        time.sleep(dt)
    send_command("rc 0 0 0 0")

def move_vertical(distance, vertical_speed=30, dt=0.1):
    """
    Adjust altitude by a specified distance (cm) in small increments.
    Positive moves up; negative moves down.
    """
    t_total = abs(distance) / vertical_speed
    iterations = int(t_total / dt)
    print("Moving vertical {:.2f} cm (expected time: {:.2f} sec, {} iterations)".format(distance, t_total, iterations))
    for _ in range(iterations):
        if distance > 0:
            send_command("rc 0 0 {} 0".format(vertical_speed))
        else:
            send_command("rc 0 0 {} 0".format(-vertical_speed))
        time.sleep(dt)
    send_command("rc 0 0 0 0")

# ---------------------------
# Interactive Control Loop
# ---------------------------
def interactive_loop():
    """
    Input a relative displacement as "x y z" (in cm) in the drone's body frame:
      - x: lateral (right positive)
      - y: forward (positive)
      - z: vertical (up positive)
    The routine:
      1. Computes the desired heading (atan2(x, y)),
      2. Rotates to face the target direction,
      3. Moves forward the horizontal distance,
      4. Adjusts altitude,
      5. Rotates back to the original heading.
    """
    while True:
        user_input = input("Enter relative displacement as 'x y z' (or 'q' to quit): ").strip()
        if user_input.lower() == 'q':
            break
        try:
            dx, dy, dz = map(float, user_input.split())
        except Exception as e:
            print("Invalid input. Please enter three numbers (cm) separated by spaces.")
            continue

        # In Tello's body frame: forward is +y, right is +x.
        desired_heading = math.degrees(math.atan2(dx, dy))
        horizontal_distance = math.sqrt(dx**2 + dy**2)
        vertical_distance = dz

        print("Desired heading: {:.2f}°".format(desired_heading))
        print("Horizontal distance: {:.2f} cm".format(horizontal_distance))
        print("Vertical distance: {:.2f} cm".format(vertical_distance))

        # Rotate to face the target direction.
        rotate_by(desired_heading)
        # Move forward for the computed horizontal distance.
        move_forward(horizontal_distance)
        # Adjust altitude as needed.
        move_vertical(vertical_distance)
        # Rotate back by the negative of the desired heading.
        rotate_by(-desired_heading)

# ---------------------------
# Main Function
# ---------------------------
def main():
    # Start the state receiver thread (for debug or future closed-loop use).
    state_thread = threading.Thread(target=state_receiver, daemon=True)
    state_thread.start()
    # Start the video thread.
    video_thread_handle = threading.Thread(target=video_thread, daemon=True)
    video_thread_handle.start()

    # Initialize SDK mode and the video stream.
    send_command("command")
    time.sleep(1)
    send_command("streamon")
    time.sleep(1)
    
    # Take off and allow stabilization.
    send_command("takeoff")
    time.sleep(5)
    
    # Enter the continuous interactive control loop.
    interactive_loop()

    # Land when done.
    send_command("land")
    cmd_sock.close()
    state_sock.close()

if __name__ == '__main__':
    main()
