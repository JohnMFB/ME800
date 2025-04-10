import threading
import socket
import time

# Local IP and port for the PC side
host = ''
port = 9000
locaddr = (host, port)

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Tello's IP and command port
tello_address = ('192.168.10.1', 8889)

sock.bind(locaddr)

def recv():
    while True:
        try:
            data, _ = sock.recvfrom(1518)
            print("Received:", data.decode("utf-8"))
        except Exception as e:
            print('Exiting receive thread:', e)
            break

print('\n\nTello Python3 Demo.\n')
print('Available commands: command, takeoff, land, battery?, lift, etc.\n')
print('Type "end" to quit.\n')

# Start receive thread as a daemon so it closes with the main thread
recvThread = threading.Thread(target=recv)
recvThread.daemon = True
recvThread.start()

def send_command(command):
    print("Sending:", command)
    sock.sendto(command.encode("utf-8"), tello_address)
    time.sleep(2)

# Step 1: Automatically enter SDK mode
send_command("command")

# Step 2: Query the battery to confirm communication
send_command("battery?")

# Listen for further commands
while True:
    try:
        msg = input("Enter command: ")
        if not msg:
            continue
        if msg.lower() == "end":
            print("Ending demo...")
            sock.close()
            break
        elif msg.lower() == "lift":
            # Revised "lift" sequence:
            send_command("takeoff")
            time.sleep(5)  # Wait for the drone to stabilize in the air
            send_command("speed 10")  # Set a slow speed
            send_command("up 50")     # Ascend 50 cm
            time.sleep(3)             # Hover at the peak
            send_command("down 50")   # Descend 50 cm
            send_command("land")
        else:
            send_command(msg)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting...")
        sock.close()
        break
