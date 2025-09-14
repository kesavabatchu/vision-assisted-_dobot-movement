import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove,DobotApiFeedBack, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re
from checkingpic import get_detected_object_center
from rotation_matrix__tranvector import compute_transformation

# 全局变量(当前坐标)
current_actual = [-1]
algorithm_queue = -1
enableStatus_robot = -1
robotErrorState = False
robotMode = 0   
globalLockValue = threading.Lock()

points_camera = [
         [-0.0714,  0.20509,     0.6600],
        [-0.08709,   0.206186,    0.8060],
        [-0.346973,  0.2025307,   1.0290]
    ]
points_base = [
         [0,      -0.354,     0],
        [0,      -0.204,    0],
        [  -0.204,  0,      0]
    ]

R,T =compute_transformation(points_camera, points_base)
def ConnectRobot():
    try:
        ip = "192.168.1.6"
        dashboardPort = 29999
        movePort = 30003
        feedPort = 30004
        print("正在建立连接...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        move = DobotApiMove(ip, movePort)
        feed = DobotApi(ip, feedPort)
        feedFour = DobotApiFeedBack(ip,feedPort)
        print(">.<连接成功>!<")
        return dashboard, move, feed,feedFour
    except Exception as e:
        print(":(连接失败:(")
        raise e


def RunPoint(move: DobotApiMove, point_list: list):
    move.MovJ(point_list[0], point_list[1], point_list[2],
              point_list[3], point_list[4], point_list[5])


def GetFeed(feedFour: DobotApiFeedBack):
    global current_actual
    global algorithm_queue
    global enableStatus_robot
    global robotErrorState
    global robotMode
# 获取机器人状态
    while True:
        with globalLockValue:
            feedInfo = feedFour.feedBackData()
            if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
                # Refresh Properties
                robotMode=feedInfo['robot_mode'][0]
                current_actual = feedInfo["tool_vector_actual"][0]
                algorithm_queue = feedInfo['run_queued_cmd'][0]
                enableStatus_robot = feedInfo['enable_status'][0]
                robotErrorState = feedInfo['error_status'][0]
                # 自定义添加所需反馈数据
            sleep(0.001)

def WaitArrive(point_list):
    while True:
        is_arrive = True
        globalLockValue.acquire()
        if current_actual is not None:
            for index in range(4):
                if (abs(current_actual[index] - point_list[index]) > 1):
                    is_arrive = False
            if is_arrive:
                globalLockValue.release()
                return
        globalLockValue.release()
        sleep(0.001)


def ClearRobotError(dashboard: DobotApiDashboard):
    global robotErrorState
    dataController, dataServo = alarmAlarmJsonFile()    # 读取控制器和伺服告警码
    while True:
        globalLockValue.acquire()
        if robotErrorState:
            numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
            numbers = [int(num) for num in numbers]
            if (numbers[0] == 0):
                if (len(numbers) > 1):
                    for i in numbers[1:]:
                        alarmState = False
                        if i == -2:
                            print("机器告警 机器碰撞 ", i)
                            alarmState = True
                        if alarmState:
                            continue
                        for item in dataController:
                            if i == item["id"]:
                                print("机器告警 Controller errorid", i,
                                      item["zh_CN"]["description"])
                                alarmState = True
                                break
                        if alarmState:
                            continue
                        for item in dataServo:
                            if i == item["id"]:
                                print("机器告警 Servo errorid", i,
                                      item["zh_CN"]["description"])
                                break

                    choose = input("输入1, 将清除错误, 机器继续运行: ")
                    if int(choose) == 1:
                        dashboard.ClearError()
                        sleep(0.01)
                        dashboard.Continue()

        else:
            if int(enableStatus_robot) == 1 and int(algorithm_queue) == 0:
                dashboard.Continue()
        globalLockValue.release()
        sleep(5)


if __name__ == '__main__':
    dashboard, move, feed,feedFour = ConnectRobot()
    dashboard.SpeedFactor(10)
    feed_thread = threading.Thread(target=GetFeed, args=(feedFour,))
    feed_thread.daemon = True
    feed_thread.start()
    feed_thread1 = threading.Thread(target=ClearRobotError, args=(dashboard,))
    feed_thread1.daemon = True
    feed_thread1.start()
    print("开始使能...")
    dashboard.EnableRobot()
    print("dobot is enabled")
    print("now it is ready to use")
    center_det=get_detected_object_center()
    if center_det is None:
        print ("nothing is detected")
    else :
        point_got = np.array(center_det)
        point_tomove = R @ point_got + T
        point_tomove*=1000
        print(point_tomove)
        point_tomove[-1]+=100
        my_list = point_tomove.tolist()
        
        my_list.append(180)
        my_list.append(0)
        my_list.append(0)
        print(my_list)
        point_a = [-200,-583.6,472.68,157.93,-8.736,154.08]#[600,-260,380,170,12,140]#
        point_b = [-122.3,-449.8,712.86,171.96,-9.46,151.86]#[200,-260,380,170,12,140]#
        pointc=[-276.1,-222.2,1044.9,-89.11,25.681,-177.3]
        pointd=[-25.82,-221.9,1090.5,-90.14,8.1912,179.44]
        point_e=[-539.1344,-373.4648,476.8670,180,0,0]


        # point_b = [-118, -576, -75, -176, 3, 162]#[-118, -576, -75, -176, 3, 162]
        RunPoint(move,point_a)
        WaitArrive(point_a)
        print("reeached point_a")
        """RunPoint(move,my_list)
        WaitArrive(my_list)"""
        print("reached to the destination")
        RunPoint(move,point_b)
        WaitArrive(point_b)
        print("back to home position")
       
        

             