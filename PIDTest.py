import threading
import socket
import time

current_state = {'x': 0, 'y': 0, 'z': 0}

# Tello Setup
TELLO_IP = '192.168.10.1'
TELLO_CMD_PORT = 8889
COMMAND_ADDRESS = (TELLO_IP, TELLO_CMD_PORT)

cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind(('', 9000))  # Local binding, adjust port as needed

state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_sock.bind(('', 8890))

# Command Test
def send_command(command):
    """Send a command to Tello and print it for debugging."""
    print("Sending:", command)
    cmd_sock.sendto(command.encode('utf-8'), COMMAND_ADDRESS)
    time.sleep(0.1)  # Small delay to allow processing

def state_receiver():
    """Continuously receive state messages and update current_state."""
    while True:
        try:
            data, _ = state_sock.recvfrom(1024)
            state_str = data.decode('utf-8').strip()

            for part in state_str.split(';'):
                if part:
                    try:
                        key, value = part.split(':')
                        # Altitude (h)
                        if key == 'h':
                            current_state['z'] = int(value)
                        # Mission Pad Coord if Present
                        elif key in ['x', 'y']:
                            current_state[key] = int(value)
                    except Exception as e:
                        print("Parsing error:", e)
            print("Current State:", current_state)
        except Exception as e:
            print("State receiver error:", e)
            break

def main():
    state_thread = threading.Thread(target=state_receiver, daemon=True)
    state_thread.start()
    # SDK Mode
    send_command("command")
    time.sleep(1)

    send_command("takeoff")
    time.sleep(5) 

    input("Press Enter to save the current position as the target... ")
    saved_position = current_state.copy()
    print("Saved Position:", saved_position)
    
    # Now, for testing, you can issue a command to go back to that saved position.
    # Use the 'go x y z speed' command. Make sure your speed (cm/s) is within [10, 100].
    # Note: This command is based on Tello's internal coordinate system,
    # and in many cases only works properly if mission pad detection is active.
    command_to_go = "go {} {} {} {}".format(saved_position.get('x', 0),
                                             saved_position.get('y', 0),
                                             saved_position.get('z', 0),
                                             20)  # Example speed = 20 cm/s
    print("Commanding drone to go to the saved position...")
    send_command(command_to_go)

    time.sleep(10)
    send_command("land")
    

    cmd_sock.close()
    state_sock.close()

if __name__ == '__main__':
    main()
