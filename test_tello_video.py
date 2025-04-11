import socket, time

TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
local_port = 9000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', local_port))

def send(cmd):
    sock.sendto(cmd.encode('utf-8'), (TELLO_IP, TELLO_PORT))
    print("Sent:", cmd)
    time.sleep(1)
    # Receive response
    try:
        data, _ = sock.recvfrom(1024)
        print("Response:", data.decode('utf-8'))
    except Exception as e:
        print("No response:", e)

send("command")
send("streamon")
