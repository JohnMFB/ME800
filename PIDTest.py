import threading
import socket
import time

# Global state dictionary to hold current Tello state.
current_state = {'x': 0, 'y': 0, 'z': 0}

# Global variable for the target (desired) position.
target_state = None

# Tello Setup
TELLO_IP = '192.168.10.1'
TELLO_CMD_PORT = 8889
COMMAND_ADDRESS = (TELLO_IP, TELLO_CMD_PORT)

# Socket for sending commands.
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind(('', 9000))  # Local binding, adjust port as needed

# Socket for receiving state feedback.
state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_sock.bind(('', 8890))

def send_command(command):
    """Send a command to Tello and print it for debugging."""
    print("Sending:", command)
    cmd_sock.sendto(command.encode('utf-8'), COMMAND_ADDRESS)
    time.sleep(0.1)  # Brief delay for processing

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
                        # Capture altitude (h) and mission pad coordinates (if available)
                        if key == 'h':
                            current_state['z'] = int(value)
                        elif key in ['x', 'y']:
                            current_state[key] = int(value)
                    except Exception as e:
                        print("Parsing error:", e)
            # Optionally print current state for debugging purposes
            print("Current State:", current_state)
        except Exception as e:
            print("State receiver error:", e)
            break

def interactive_loop():
    """Interactive loop to capture or input target positions repeatedly."""
    global target_state

    while True:
        print("\n--- Interactive Command ---")
        print("Options:")
        print("   c  : Capture current position as the target")
        print("   m  : Manually input target coordinates (x y z in cm)")
        print("   q  : Quit and land")
        user_input = input("Enter your choice (c/m/q): ").strip().lower()

        if user_input == 'q':
            break
        elif user_input == 'c':
            # Capture the live current state as the target position.
            target_state = current_state.copy()
            print("Captured Target Position:", target_state)
        elif user_input == 'm':
            try:
                coords = input("Enter target coordinates as x y z (in cm): ").strip()
                x, y, z = map(int, coords.split())
                target_state = {'x': x, 'y': y, 'z': z}
                print("Manually set Target Position:", target_state)
            except Exception as e:
                print("Invalid input. Please enter three integer values separated by spaces.")
                continue
        else:
            print("Invalid option. Please try again.")
            continue

        # Check that target_state is set; if not, warn the user.
        if target_state is None:
            print("No target position defined. Please capture or input a target.")
            continue

        # Prepare the "go" command.
        # Note: 'go x y z speed' is more effective with mission pad detection.
        speed = 20  # Example speed in cm/s; adjust if needed.
        command_to_go = "go {} {} {} {}".format(
            target_state.get('x', 0),
            target_state.get('y', 0),
            target_state.get('z', 0),
            speed
        )
        print("Commanding drone to go to the target position...")
        send_command(command_to_go)

        # Optionally, wait here until the drone should have reached the target.
        # This wait time depends on the distance and your speed.
        time.sleep(10)

        # Reset the target_state to avoid using stale data.
        target_state = None

def main():
    # Start state receiver thread.
    state_thread = threading.Thread(target=state_receiver, daemon=True)
    state_thread.start()

    # Enter SDK mode.
    send_command("command")
    time.sleep(1)

    # Take off.
    send_command("takeoff")
    time.sleep(5)  # Allow time to stabilize.

    # Enter interactive loop.
    interactive_loop()

    # Land when done.
    send_command("land")
    cmd_sock.close()
    state_sock.close()

if __name__ == '__main__':
    main()
