import threading
import socket
import time
import math

current_state = {'x': 0, 'y': 0, 'z': 0, 'yaw': 0}

TELLO_IP = '192.168.10.1'
TELLO_CMD_PORT = 8889
COMMAND_ADDRESS = (TELLO_IP, TELLO_CMD_PORT)

cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind(('', 9000))  

state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_sock.bind(('', 8890))

def send_command(command):
    print("Sending:", command)
    cmd_sock.sendto(command.encode('utf-8'), COMMAND_ADDRESS)
    time.sleep(0.1) 

def state_receiver():
    while True:
        try:
            data, _ = state_sock.recvfrom(1024)
            state_str = data.decode('utf-8').strip()
            for part in state_str.split(';'):
                if part:
                    try:
                        key, value = part.split(':')
                        if key == 'h':
                            current_state['z'] = int(value)
                        elif key in ['x', 'y']:
                            current_state[key] = int(value)
                        elif key == 'yaw':
                            current_state['yaw'] = int(value)
                    except Exception as e:
                        print("Parsing error:", e)
            print("Current State:", current_state)
        except Exception as e:
            print("State receiver error:", e)
            break

def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def interactive_loop():

    while True:
        print("\n--- Interactive Command ---")
        print("Options:")
        print("   m  : Manually input a relative target displacement (dx dy dz in cm)")
        print("         (dx,dy define horizontal displacement; dz is altitude change)")
        print("   q  : Quit and land")
        choice = input("Enter your choice (m/q): ").strip().lower()
        if choice == 'q':
            break
        elif choice == 'm':
            try:
                coords = input("Enter target displacement as dx dy dz (in cm): ").strip()
                dx, dy, dz = map(int, coords.split())
                rel_target = {'dx': dx, 'dy': dy, 'dz': dz}
                print("Manually set Relative Target:", rel_target)
            except Exception as e:
                print("Invalid input. Please enter three integer values separated by spaces.")
                continue
        else:
            print("Invalid option. Try again.")
            continue

        # Capture the starting state.
        start_state = current_state.copy()
        original_yaw = start_state.get('yaw', 0)
        
        # Compute the desired heading angle from relative displacement.
        if rel_target['dx'] == 0 and rel_target['dy'] == 0:
            desired_heading = original_yaw
            horizontal_distance = 0
        else:
            desired_heading = math.degrees(math.atan2(rel_target['dy'], rel_target['dx']))
            horizontal_distance = int(round(math.sqrt(rel_target['dx']**2 + rel_target['dy']**2)))
        yaw_change = normalize_angle(desired_heading - original_yaw)
        
        print("Original Yaw:", original_yaw)
        print("Desired Heading:", desired_heading)
        print("Yaw Change:", yaw_change)
        print("Horizontal Distance to move:", horizontal_distance)

    
        if yaw_change > 0:
            send_command("cw {}".format(int(round(yaw_change))))
        elif yaw_change < 0:
            send_command("ccw {}".format(int(round(abs(yaw_change)))))

        time.sleep(2) # Required Delay

        if horizontal_distance > 0:
            send_command("forward {}".format(horizontal_distance))
        
        if rel_target['dz'] > 0:
            send_command("up {}".format(rel_target['dz']))
        elif rel_target['dz'] < 0:
            send_command("down {}".format(abs(rel_target['dz'])))
        
        time.sleep(5)
        
        if yaw_change > 0:
            send_command("ccw {}".format(int(round(yaw_change))))
        elif yaw_change < 0:
            send_command("cw {}".format(int(round(abs(yaw_change)))))
        
        # Allow a short delay after rotating back.
        time.sleep(2)

def main():
    # Start the state receiver thread.
    state_thread = threading.Thread(target=state_receiver, daemon=True)
    state_thread.start()

    # Enter SDK mode.
    send_command("command")
    time.sleep(1)
    
    send_command("takeoff")
    time.sleep(5)

    interactive_loop()

    send_command("land")
    cmd_sock.close()
    state_sock.close()

if __name__ == '__main__':
    main()
