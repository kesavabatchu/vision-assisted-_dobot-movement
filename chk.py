
import time
import keyboard  # Install with: pip install keyboard
from dobot_api import DobotApiDashboard

# Initialize Dobot over Ethernet
def initialize_dobot(ip_address):
    try:
        dashboard = DobotApiDashboard(ip_address, 29999)
        dashboard.ClearError()
        dashboard.EnableRobot()
        time.sleep(1)
        print("Dobot Nova 5 initialized and ready.")
        return dashboard
    except Exception as e:
        print(f"Error initializing Dobot: {e}")
        return None

# Move Dobot with keyboard
def keyboard_control(dashboard):
    x, y, z, rx, ry, rz = 200, -300, 150, 0, 0, 0  # Initial position

    print("Use W/A/S/D for X-Y movement, Q/E for Z, Arrow Keys for Rx, Ry, Rz")
    print("Press ESC to exit.")

    while True:
        if keyboard.is_pressed('w'):  # Move +X
            x += 10
        elif keyboard.is_pressed('s'):  # Move -X
            x -= 10
        elif keyboard.is_pressed('a'):  # Move -Y
            y -= 10
        elif keyboard.is_pressed('d'):  # Move +Y
            y += 10
        elif keyboard.is_pressed('q'):  # Move +Z
            z += 10
        elif keyboard.is_pressed('e'):  # Move -Z
            z -= 10
        elif keyboard.is_pressed('up'):  # Rotate +Rx
            rx += 5
        elif keyboard.is_pressed('down'):  # Rotate -Rx
            rx -= 5
        elif keyboard.is_pressed('left'):  # Rotate -Ry
            ry -= 5
        elif keyboard.is_pressed('right'):  # Rotate +Ry
            ry += 5
        elif keyboard.is_pressed('shift'):  # Rotate +Rz
            rz += 5
        elif keyboard.is_pressed('ctrl'):  # Rotate -Rz
            rz -= 5
        elif keyboard.is_pressed('esc'):  # Exit program
            print("Exiting...")
            break
        
        command = f"MovL({x},{y},{z},{rx},{ry},{rz})"
        dashboard.sendRecvMsg(command)
        time.sleep(0.2)  # Small delay to avoid overloading commands

def main():
    ip_address = "192.168.5.1"  # Update this with your actual Dobot IP
    dashboard = initialize_dobot(ip_address)
    if dashboard:
        keyboard_control(dashboard)
        dashboard.close()

if __name__ == "__main__":
    main()
