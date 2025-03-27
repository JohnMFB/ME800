from djitellopy import Tello
import time

def main():
    # Create the Tello object
    tello = Tello()
    
    # Try connecting multiple times
    max_attempts = 5
    connected = False
    attempts = 0

    while not connected and attempts < max_attempts:
        try:
            print(f"Attempting to connect to Tello (attempt {attempts+1})...")
            tello.connect()  # Sends the 'command' to enter SDK mode
            print("Connected! Battery level:", tello.get_battery(), "%")
            connected = True
        except Exception as e:
            print(f"Connection attempt {attempts+1} failed: {e}")
            attempts += 1
            print("Waiting 5 seconds before retrying...")
            time.sleep(5)

    if not connected:
        print("Failed to connect after multiple attempts. Please check the drone's status, firmware, and battery.")
    else:
        # Continue with further commands if needed
        print("Tello is now connected and ready for commands.")
        # For example, you could eventually send a takeoff command:
        # tello.takeoff()
        # time.sleep(5)
        # tello.land()

if __name__ == "__main__":
    main()
