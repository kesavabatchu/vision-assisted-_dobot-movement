from dobot_api import DobotApiDashboard
import time

def test_DO1():
    # Connect to Dobot Nova 5 controller
    dash = DobotApiDashboard('192.168.1.6', 29999)
    time.sleep(0.2)
    
    # Enable robot (required for tool I/O)
    dash.EnableRobot()
    time.sleep(0.2)
    
    # Set DO1 high (index 0 → DO1)
    dash.DOExecute(1, 1)
    print("DO1 set to HIGH. Measure voltage between DO1 and COM now.")
    time.sleep(3)
    
    # Reset DO1 low
    dash.DOExecute(1, 0)
    print("DO1 set to LOW. Test complete.")
    
    # Disable robot
    dash.DisableRobot()

if __name__ == '__main__':
    test_DO1()
