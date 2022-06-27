#!/usr/bin/env python3
# encoding: utf-8
import sys
import os
import LeActList
import threading
import Serial_Servo_Running
import time
import Board


actdir = "/home/pi/human_code/ActionGroups/"
# 校准值
Calibration = (500, 500, 303, 500, 500, 500, 500, 725, 500, 500, 697, 500, 500, 500, 500, 275)


class LeError(Exception):
    def __init__(self, data=(), msg="LeError"):
        self.data = data
        self.msg = msg


def cmd_i000(par):  # 指令000 空指令
    pass


def cmd_i001(par):  # 指令001 舵机运动
    global Deviation
    if par[0] > 30000:  # 时间限制
        par[0] = 30000
    if par[0] < 20:
        par[0] = 20
    #
    if not par[1] * 2 + 2 == len(par) or not len(par) >= 4:
        raise LeError(tuple(par), "舵机运动指令长度错误")
    Servos = par[2:]
    for i in range(0, len(Servos), 2):
        if Servos[i] > 16 or Servos[i] < 1 or Servos[i+1] > 1000 or Servos[i+1] < 0:
            raise LeError((Servos[i], Servos[i+1]), "舵机运动参数错误")
        # 串口舵机
        Board.setBusServoPulse(Servos[i], Servos[i + 1], par[0])


def cmd_i002(par):  # 指令002 停止运动
    Serial_Servo_Running.stopActionGroup()


def cmd_i003(sock, data=["", ""]):  # 指令003 运行动作组
    if (not len(data) == 2) or (not data) or (not data[0]) or (not data[1]) :
        raise LeError(tuple(data), "运行动作组指令错误")
    par = None
    try:
        par = int(data[1])
    except:
        raise LeError((data[1],),"动作组运行次数错误")

#    print(data[0])
#    print(par)
    if not par is None:
        try:
            Serial_Servo_Running.runAction(data[0])
        except Exception as e:
            print(e)

def cmd_i004(sock, data=[]):  # 指令 004查询动作组
    actList = LeActList.listActions(actdir)
    actList.sort()
    if not len(actList) is 0:
        
        for i in range(0, len(actList), 10):
            str_head = "I004-" + str(len(actList))
            str_tial = "-" + str(i+1) + "-"
            str_tial1 = ""
            t = 10
            for j in range(0, 10, 1):
                if i+j < len(actList):
                    str_tial1 += "-" + actList[i+j][:-4]
                else:
                    if t == 10:
                        t = j
            if str_tial1:
                str_head = str_head + str_tial + str(i+t) + str_tial1 + "\r\n"
                sock.sendall(str_head.encode())
    else:
        s = "I004-0-0-0\r\n"
        sock.sendall(s.encode())

#    print(len(actList))
#    print(actList)


def cmd_i005(sock, data=[]):  # 指令 005 删除一个动作组
    if data:
        for d in data:
            if d:
                os.remove(actdir + d + ".d6a")


def cmd_i006(sock, data=[]):  # 指令 006 删除所有动作组
    actList = LeActList.listActions(actdir)
    for d in actList:
        os.remove(actdir + d)


def cmd_i007(sock, data=[]):    # 手柄控制
    try:
        time1 = int(data[0])
        servo_num = int(data[1])
        # print(time,servo_num)
        servo_data = []
        for i in range(servo_num):
            servo_id = int(data[2 + i * 2])
            servo_pos = int(data[3 + i * 2])
            servo_data.append((servo_id, servo_pos-10000))
        # print(servo_data)
    except:
        raise LeError(tuple(data), "参数错误")
    try:
        for d in servo_data:
            Board.setBusServoPulse(d[0], d[1], time1)
            time.sleep(0.5)
    except Exception as e:
        print(e)


def read_deviation():   # 读取舵机内部偏差
    d = []
    for i in range(1, 17, 1):
        zf_d = Board.getBusServoDeviation(i)
        if zf_d > 127:  # 负数
            zf_d = -(0xff - (zf_d - 1))
        d.append(zf_d)
    return d


def cmd_i008(sock, data=[]):      # 修改偏差
    # 1、读取界面舵机P值
    if not 32 == len(data):
        raise LeError(tuple(data), "1、舵机运动指令长度错误")
    Servos = data[:]
    j = 0
    upper_d = []    # 界面的舵机值
    for i in range(0, len(Servos), 2):
        j += 1
        upper_d.append(int(Servos[i+1]))
    ########################################################
    if not len(Calibration) == 16 and not len(upper_d) == 16:
        print("偏差数量错误")
        s = "I008-dev-no\r\n"
        sock.sendall(s.encode())
        sys.exit()
    else:
        new_d = []
        for i in range(0, len(upper_d), 1):
            if -125 > (upper_d[i] - Calibration[i]) > 125:
                print("偏差值超出范围-125~125")
                s = "I008-dev-error-range\r\n"
                sock.sendall(s.encode())
                sys.exit()
            else:
                # 界面值 - 校准值
                new_d.append(upper_d[i] - Calibration[i])
    # 配置偏差
    for i in range(0, len(new_d), 1):
        Board.setBusServoDeviation(i+1, new_d[i])
        time.sleep(0.05)
        Board.saveBusServoDeviation(i+1)
    s = "I008-dev-ok\r\n"
    sock.sendall(s.encode())     # 发送修改偏差成功


def cmd_i009(sock, data=[]):  # 读取舵机内部偏差
    global Deviation
    # 1、读取舵机内部偏差值
    Deviation = read_deviation()
    # 2、清零舵机内部偏差
    for i in range(0, 16, 1):
        Board.setBusServoDeviation(i+1, 0)
        time.sleep(0.05)
        Board.saveBusServoDeviation(i+1)
        time.sleep(0.05)
    # 3、计算校准值 + 偏差值，生成字符串命令
    str_head = "I009-"
    str_head += str(len(Deviation))
    str_tial = ''
    for i in range(0, len(Deviation), 1):
        str_tial += "-" + str(i+1) + "-" + str(Calibration[i] + Deviation[i])
    str_head += str_tial
    # 4、发送数据给上位机
    sock.sendall(str_head.encode())


cmd_list = [cmd_i000, cmd_i001, cmd_i002, cmd_i003, cmd_i004, cmd_i005, cmd_i006,
            cmd_i007, cmd_i008, cmd_i009]

