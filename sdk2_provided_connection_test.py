import threading
import socket
import time

host = ''
port = 9000
locaddr = (host, port)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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

recvThread = threading.Thread(target=recv)
recvThread.daemon = True
recvThread.start()

def send_command(command):
    print("Sending:", command)
    sock.sendto(command.encode("utf-8"), tello_address)
    time.sleep(2)

send_command("command")

send_command("battery?")

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
    
            send_command("takeoff")
            time.sleep(5) 
            send_command("speed 10")  
            send_command("up 50")    
            time.sleep(3)            
            send_command("down 50") 
            send_command("land")
        else:
            send_command(msg)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting...")
        sock.close()
        break
