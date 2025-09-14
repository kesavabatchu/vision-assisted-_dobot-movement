import threading
from dobot_api import DobotApiDashboard, DobotApiMove, DobotApiFeedBack
from time import sleep

current_actual = [-1]
globalLockValue = threading.Lock()

def ConnectRobot():
    ip = "192.168.1.6"
    dashboardPort = 29999
    movePort = 30003
    feedPort = 30004

    print("Connecting...")
    dashboard = DobotApiDashboard(ip, dashboardPort)
    move = DobotApiMove(ip, movePort)
    feed = DobotApiFeedBack(ip, feedPort)
    print("Connected!")
    return dashboard, move, feed

def GetFeed(feed: DobotApiFeedBack):
    global current_actual
    while True:
        with globalLockValue:
            feedInfo = feed.feedBackData()
            print(feedInfo.dtype.names)  # to get all field names
            # For now, let's print the first element's relevant fields to identify tool_vector_actual
            print(feedInfo[0])  
        sleep(0.1)

def WaitArrive(target_pos):
    while True:
        with globalLockValue:
            if current_actual == [-1]:
                continue
            # Check if all xyzr match within tolerance
            if all(abs(current_actual[i] - target_pos[i]) < 5 for i in range(6)):
                return
        sleep(0.1)

if __name__ == "__main__":
    dashboard, move, feed = ConnectRobot()

    # Start feedback thread to update current_actual
    feed_thread = threading.Thread(target=GetFeed, args=(feed,), daemon=True)
    feed_thread.start()

    print("Homing robot...")
    dashboard.ClearError()
    sleep(5)  # wait for homing to complete, adjust if needed

    print("Enabling robot...")
    dashboard.EnableRobot()
    sleep(1000)
    safe_point = [100, 0, 100, 0, 0, 0]
    move.MovL(*safe_point)

    # Print current actual position 5 times as a sanity check
    for _ in range(5):
        with globalLockValue:
            print(f"Current Position: {current_actual}")
        sleep(1)

    # Define a safe position (adjust if needed after manual tests)
    safe_point = [100, 0, 100, 0, 0, 0]

    print(f"Moving to safe point: {safe_point}")
    move.MovL(*safe_point)
    WaitArrive(safe_point)

    print("Arrived at safe point. Program finished.")
