from djitellopy import Tello
import time

def main():
    print("Attempting to connect...")
    tello = Tello()
    tello.connect()
    print("Connected! Battery:", tello.get_battery(), "%")
    
    # Wait to let state packets stabilize
    time.sleep(5)
    
    input("Press Enter to take off...")
    tello.takeoff()
    time.sleep(5)
    input("Press Enter to land...")
    tello.land()
    print("Finished.")

if __name__ == "__main__":
    main()
