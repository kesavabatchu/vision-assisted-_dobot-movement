from dobot_api import DobotApiDashboard, DobotApi, DobotApiFeedBack
import threading
import time

# Global variables for DI status
digital_input_bits = None
globalLockValue = threading.Lock()
#dashboard = DobotApiDashboard('192.168.1.6', 29999)  # Replace with your Dobot's IP

def ConnectRobot():
    """Connect to Dobot Nova 5 robot controller"""
    try:
        ip = "192.168.1.6"
        dashboardPort = 29999
        feedPort = 30004
        print("Establishing connection...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        feed = DobotApiFeedBack(ip, feedPort)
        print("Connection successful!")
        return dashboard, feed
    except Exception as e:
        print("Connection failed:", e)
        raise e

def GetDigitalInputs(feed: DobotApiFeedBack):
    """Continuously read digital input status from robot feedback"""
    global digital_input_bits
    
    while True:
        try:
            with globalLockValue:
                feedInfo = feed.feedBackData()
                if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
                    # Get digital input bits from feedback
                    digital_input_bits = feedInfo['digital_input_bits'][0]
            time.sleep(0.01)  # 10ms polling rate
        except Exception as e:
            print(f"Error reading feedback: {e}")
            time.sleep(0.1)

def read_digital_input(di_channel):
    """
    Read the status of a specific Digital Input (DI) channel
    di_channel: int, the DI number to check (1-8 for Nova 5)
    Returns: int (0 for LOW, 1 for HIGH) or None if error
    """
    global digital_input_bits
    
    if digital_input_bits is None:
        print("Digital input data not available yet")
        return None
        
    if di_channel < 1 or di_channel > 8:
        print("Invalid DI channel. Valid range: 1-8")
        return None
    
    with globalLockValue:
        # Extract the specific bit for the requested DI channel
        # DI channels are numbered 1-8, but bits are 0-indexed
        bit_position = di_channel - 1
        di_status = (digital_input_bits >> bit_position) & 1
        
    return di_status
def gripper_open(dashboard):
    """Open the PGE 100-26 gripper completely"""
    # Set appropriate digital outputs to open gripper
    dashboard.DOExecute(1, 1)  # Set DO1 to HIGH (adjust pin as needed)
    #dashboard.ToolDO(1, 1)  # Set DO2 to LOW (adjust pin as needed)
    print("Gripper opening...")
    time.sleep(1)  # Wait 1 second for gripper to open

def gripper_close(dashboard):
    """Close the PGE 100-26 gripper completely"""
    # Set appropriate digital outputs to close gripper
    dashboard.DOExecute(1, 0)  # Set DO1 to LOW
    #dashboard.ToolDO(1, 0)  # Set DO2 to HIGH
    print("Gripper closing...")
    time.sleep(1.5)
def gripper_stop(dashboard):
    """Stop gripper movement"""
    dashboard.DOExecute(1, 0)  # Set DO1 to LOW
    #dashboard.ToolDO(1, 0)  # Set DO2 to LOW
    print("Gripper stopped")
def read_all_digital_inputs():
    """
    Read the status of all Digital Input channels
    Returns: dict with DI channel numbers as keys and status as values
    """
    global digital_input_bits
    
    if digital_input_bits is None:
        print("Digital input data not available yet")
        return None
    
    di_statuses = {}
    with globalLockValue:
        for channel in range(1, 9):  # DI1 to DI8
            bit_position = channel - 1
            status = (digital_input_bits >> bit_position) & 1
            di_statuses[f"DI{channel}"] = status
    
    return di_statuses

def monitor_digital_input(di_channel, callback_func=None):
    """
    Monitor a specific DI channel and call callback function on state change
    di_channel: int, the DI channel to monitor
    callback_func: function to call when state changes (optional)
    """
    previous_state = None
    
    while True:
        current_state = read_digital_input(di_channel)
        
        if current_state is not None and current_state != previous_state:
            print(f"DI{di_channel} changed: {previous_state} -> {current_state}")
            
            if callback_func:
                callback_func(di_channel, current_state, previous_state)
                
            previous_state = current_state
        
        time.sleep(0.1)  # Check every 100ms

def main():
    """Main function to demonstrate DI reading"""
    try:
        # Connect to robot
        dashboard, feed = ConnectRobot()
        
        # Enable robot (required for I/O operations)
        dashboard.EnableRobot()
        print("Robot enabled")
        
        # Start feedback thread
        feed_thread = threading.Thread(target=GetDigitalInputs, args=(feed,))
        feed_thread.daemon = True
        feed_thread.start()
        
        # Wait a moment for feedback to start
        time.sleep(2)
        
        print("Starting DI monitoring...")
        count=0
        while True:
            # Method 1: Read a specific DI channel
            di1_status = read_digital_input(5)
            
            print(f"Digital input 5 : {di1_status}")
            count=count+1
            if count==20:
                break
            # Method 2: Read all DI channels
            # all_di = read_all_digital_inputs()
            # if all_di:
            #   print("All DI statuses:", all_di)
            
            
            
            time.sleep(1)  # Print status every 
            
    except KeyboardInterrupt:
        print("Stopping DI monitoring...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            dashboard.DisableRobot()
            print("Robot disabled")
        except:
            pass

if __name__ == '__main__':
    main()
