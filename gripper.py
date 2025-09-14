from dobot_api import DobotApiDashboard, DobotApi
import time

# Connect to Dobot
dashboard = DobotApiDashboard('192.168.1.6', 29999)  # Replace with your Dobot's IP
feed = DobotApi('192.168.1.6', 30004)  # For feedback functions

# Gripper control functions
def gripper_open():
    dashboard.DOExecute(1, 1)  # Set DO1 to HIGH (adjust pin as needed)
    print("valve opened")
    time.sleep(3)  

def gripper_close():
    
    dashboard.DOExecute(1, 0)  # Set DO1 to LOW
    
    print("valve closed")
    time.sleep(1.5)  # Wait 1.5 seconds for gripper to close

def gripper_stop():
    """Stop gripper movement"""
    dashboard.DOExecute(1, 0)  # Set DO1 to LOW
    #dashboard.ToolDOExecute(1, 0)  # Set DO2 to LOW
    print("Gripper stopped")

# Simple test program
def test_gripper():
    try:
        # Enable the robot (required for I/O control)
        dashboard.EnableRobot()
        time.sleep(3)
        print("Robot enabled - testing gripper")
        
        # Test sequence
        gripper_open()
        time.sleep(2) 
        
        gripper_close()
        #gripper_open()
        #gripper_stop()
        
        # Disable robot when done
        dashboard.DisableRobot()
        print("Gripper test complete")
        
    except Exception as e:
        print("Error:", e)
        dashboard.DisableRobot()

if __name__ == '__main__':
    test_gripper()