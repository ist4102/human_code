#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import sys
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
import cv2
import math
import time
import PWMServo
import threading
import numpy as np
from config import *
import RPi.GPIO as GPIO
from cv_ImgAddText import *
import Serial_Servo_Running as SSR

print('''
**********************************************************
*****巡线：通过摄像头检测黑线，使机器人沿着黑线运动*******
**********************************************************
----------------------------------------------------------
Official website:http://www.lobot-robot.com/pc/index/index
Online mall:https://lobot-zone.taobao.com/
----------------------------------------------------------
Version: --V2.2  2020/09/29
----------------------------------------------------------
''')

cross_line = 0 #记录走过的十字路口个数，一共9个十字路口，我们将第4和第7个十字路口舍去识别，所有一共7个
kick = False #是否踢球
bridge_up = 0 #上桥还是下桥， 0未上桥，2下完桥

#上桥黄线位置
yellow_line_y = 80

#上桥前识识别到黄线切换步态
yellow_line_y1 = 60 
yellow_line_y2 = 120

#下桥时识别的黄线位置y坐标
yellow_line_y3 = 10
yellow_line_y4 = 120

#摄像头实际识别到物体在中间时，横坐标的值
set_black_line_centerx = 78

#识别十字路口阈值
min_value = 0
max_value = 22000

min_value1 = 7
#min_value1 = 7
max_value1 = 10000

count_center = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

#使用到的动作组，存储在/home/human_code/ActionGroups/
go_forward_fast = 'go_forward'
go_forward = 'go_forward'
go_straight = go_forward_fast
go_straight_kick = 'go_forward'
turn_right = 'turn_right'
turn_left  = 'turn_left'

debug = True
servo1_ = servo1

#寻迹时摄像头舵机朝下看
PWMServo.setServo(1, servo1, 500)
PWMServo.setServo(2, servo2, 500)

#蜂鸣器io口初始化
buzzer_pin = 23
GPIO.setup(buzzer_pin, GPIO.OUT)
GPIO.output(buzzer_pin, 0)

#寻迹时，画出的线框颜色
line_color = (255, 0, 0)
#寻迹时，画出的线框的粗细
line_thickness = 2         

orgFrame = None
orgframe = None
Running = True
#图像分辨率
ori_width, ori_height = 160, 120
#调用摄像头
stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
cap = cv2.VideoCapture(stream)
def get_img():
    global orgFrame
    while True:
        if cap.isOpened():
            #读取图片
            ret, orgframe = cap.read()          
            #将摄像头画面缩小以便处理
            orgFrame = cv2.resize(orgframe, (ori_width, ori_height), interpolation = cv2.INTER_CUBIC)
        else:
            time.sleep(0.01)

threading.Thread(target=get_img, daemon=True).start()

#映射函数
def leMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#获取最大轮廓以及它的面积
def getAreaMax_contour(contours, area_min = 5):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours: #历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c)) #计算轮廓面积
            if contour_area_temp > contour_area_max :
                contour_area_max = contour_area_temp
                if contour_area_temp > area_min:  #只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c

        return area_max_contour, contour_area_max#返回最大的轮廓

roi = [ # [ROI, weight]
        (0,  20,  0, 160, 0.4), 
        (40, 60,  0, 160, 0.5), 
        (80, 100,  0, 160, 0.1)
       ]

black_line_centerx = 80 #设定的线中心
get_black_line = True
get_cross_line_up = False
get_cross_line_down = False
get_cross_line = False
print_status = True
last_center = []
#寻迹主体程序，颜色为黑色
def Tracing(orgimage, r_w, r_h, r = roi, l_c = line_color, l_t = line_thickness):
    global min_value, max_value, min_value1, max_value1
    global black_line_centerx, bridge_up, last_center
    global cross_line, get_black_line, get_cross_line
    global print_status, get_cross_line_up, get_cross_line_down
    global count_center
    
    frame_gaussianblur = cv2.GaussianBlur(orgimage, (3, 3), 0)#高斯模糊
    frame_lab = cv2.cvtColor(frame_gaussianblur, cv2.COLOR_BGR2LAB) #将图像转换到LAB空间
    mask = cv2.inRange(frame_lab, color_range['black'][0], color_range['black'][1]) #根据lab值对图片进行二值化
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))#开运算
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8))#闭运算

    n = 0
    centroid_x_sum = 0
    weight_sum = 0
    #将图像分割成上中下三个部分，这样处理速度会更快，更精确
    for r in roi:
        n += 1
        blobs = closed[r[0]:r[1], r[2]:r[3]]
        y_acc = np.sum(blobs, axis=1)
        y_index1 = np.argmax(y_acc)

        cnts = cv2.findContours(blobs, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]#找出所有轮廓
        cnt_large= getAreaMax_contour(cnts)[0]#找到最大面积的轮廓
        if cnt_large is not None:#如果轮廓不为空
            rect = cv2.minAreaRect(cnt_large)#最小外接矩形
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
            box[0, 1], box[1, 1], box[2, 1], box[3, 1] = box[0, 1] + (n - 1)*40, box[1, 1] + (n - 1)*40, box[2, 1] + (n - 1)*40, box[3, 1] + (n - 1)*40#对获取的点做处理
            
            #获取矩形的对角点
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]

            cv2.drawContours(orgframe, [box], -1, (0, 0, 255, 255), 2)#画出四个点组成的矩形
            center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2#中心点

            #print(n, y_index1, y_acc[y_index1])
            if y_index1 > min_value and y_acc[y_index1] > max_value and (bridge_up == 2 or (bridge_up != 2 and cross_line < 3)):
                if not get_cross_line:
                    #当上部分检测到十字交叉口时，我们认为开始检测到十字交叉口
                    if n == 1 or n == 2:
                        if print_status:
                            print_status = False
                            print('检测到黑色标识线')
                        if not get_cross_line_up:
                            get_cross_line_up = True
                    elif n == 3:#当下部分检测到十字交叉口时，我们认为现在处于离十字交叉口最近的地发，且处于十字交叉口前方 
                        last_center.append(centerx)
                        if len(last_center) >= 10:
                            last_center.remove(last_center[0])
                        if get_cross_line_up:
                            get_cross_line_down = True
            else:
                if get_cross_line_down and get_cross_line is False:
                    cross_line += 1
                    if not print_status:
                        print_status = True
                        print('当前处于第' + str(cross_line) + '条标识线')
                    get_cross_line_down = False
                    get_cross_line = True
                    get_cross_line_up = False
                if 10 > y_index1 > min_value1 or y_acc[y_index1] < max_value1:
                    cv2.circle(orgframe, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)#画出中心点
                    #按权重不同对上中下三个中心点进行求和
                    centroid_x_sum += center_x * r[4]
                    weight_sum += r[4]

    if weight_sum is not 0:
        #求最终得到的中心点
        black_line_centerx = int(centroid_x_sum / weight_sum)
        if trace:
            if len(count_center) == 10:
                count_center.remove(count_center[0])
                count_center.append(black_line_centerx)
       # print(black_line_centerx)
        get_black_line = True
    else:
        get_black_line = False

range_rgb = {'red': (0, 0, 255),
              'blue': (255, 0,0),
              'green': (0, 255, 0),
              'black': (0, 0, 0),
              'yellow':(255, 255, 0),
              }

color_list = []
COLOR = 'None'
cylinder_red_finish = False
cylinder_green_finish = False
cylinder_blue_finish = False
#颜色识别，主要用来识别红绿蓝三种颜色
def colors_identify(img):  
    global color_list, COLOR
    global cylinder_red_finish, cylinder_green_finish, cylinder_blue_finish

    frame_gaussianblur = cv2.GaussianBlur(img, (3,3), 0)#高斯模糊
    frame_lab = cv2.cvtColor(frame_gaussianblur, cv2.COLOR_BGR2LAB)#将图片转换到LAB空间

    max_area = 0
    color_max = 'None'
    #根据红绿蓝三种颜色的lab值进行处理
    for i in color_range:
        #去除黑色和黄色，防止干扰，提高速度
        if i != 'black' and i != 'yellow':
            #如果前面已经识别过其中的某种颜色，则此次不再识别，提高速度
            if (cylinder_red_finish == False and i == 'red') or (cylinder_green_finish == False and i == 'green') or (cylinder_blue_finish == False and i == 'blue'): 
                mask = cv2.inRange(frame_lab, color_range[i][0], color_range[i][1])#对原图像和掩模进行位运算
                opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))#开运算
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))#闭运算
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]#找出轮廓
                areaMax_contour, area_max = getAreaMax_contour(contours)#找出最大轮廓
                if areaMax_contour is not None:
                    if area_max > max_area:#找最大面积
                        max_area = area_max
                        color_max = i
                        areaMax_contour_max = areaMax_contour
    if max_area != 0:
        ((black_line_centerx, centerY), rad) = cv2.minEnclosingCircle(areaMax_contour_max)  # 获取最小外接圆
        black_line_centerx, centerY, rad = int(black_line_centerx), int(centerY), int(rad)#获取圆心，半径
        cv2.circle(orgframe, (black_line_centerx, centerY), rad, (0, 255, 0), 2)#画圆
        if color_max == 'red':  #红色最大
            color = 1
        elif color_max == 'green':  #绿色最大
            color = 2
        elif color_max == 'blue':  #蓝色最大
            color = 3
        else: 
            color = 0
        #获取最近5次的值
        color_list.append(color)
        if len(color_list) == 5:
            #取平均
            color = int(round(np.mean(color_list)))
            color_list = []
            if color == 1:
                COLOR = 'red'
            elif color == 2:
                COLOR = 'green'
            elif color == 3:
                COLOR = 'blue'
            else:
                return None
            return COLOR
        else:
            return None
    else:
        COLOR = "None"
        return None

X = 0
Y = 0 
get_ball = False
#区别与上一个，这个函数是识别特定的颜色，根据输入的颜色来识别
def color_identify(img, target_color = 'yellow'):
    global X, Y, get_ball
    
    frame_gaussianblur = cv2.GaussianBlur(img, (3, 3), 0)#高斯模糊
    frame_lab = cv2.cvtColor(frame_gaussianblur, cv2.COLOR_BGR2LAB) #将图像转换到LAB空间
    mask = cv2.inRange(frame_lab, color_range[target_color][0], color_range[target_color][1]) #根据lab值对图片进行二值化
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))#开运算
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8))#闭运算
    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2] #找出所有外轮廓
    areaMax_contour = getAreaMax_contour(contours, area_min = 3)[0] #找到最大的轮廓
    
    X = 999
    Y = 999
    if areaMax_contour is not None:  #有找到最大面积
        if target_color != 'black':
            (X, Y), radius = cv2.minEnclosingCircle(areaMax_contour) #获取最小外接圆
            X = int(X)
            Y = int(Y)
            #print(X, Y)
            radius = int(radius)
            cv2.circle(orgframe, (X, Y), radius, range_rgb[target_color], 2)
            if radius > 5:
                get_ball = True       
                return True

            else:
                return False
    else:
        X, Y = -999, -999
        return False
        
cylinder_color = None
cylinder_start = True
get_cylinder = False
#每个圆柱对应的动作
def action(color, start = True):
    global cylinder_color, go_straight, turn_left, turn_right
    global cylinder_red_finish, cylinder_green_finish, cylinder_blue_finish, cylinder_start, get_cylinder, cross_line
    #每个动作只会执行一遍，例如红色圆柱的动作执行完，即使下次再识别到红色也不会执行，防止干扰
    if color == 'red' and cylinder_red_finish is False and cylinder_start:
        if start:
            print('执行red圆柱对应动作，蜂鸣器响')
            GPIO.output(buzzer_pin, 1)
            cylinder_color = color
            cylinder_start = False
            get_cylinder = False
        else:
            print('停止red圆柱对应动作\n')
            GPIO.output(buzzer_pin, 0)
            cylinder_start = False
            cylinder_red_finish = True
    elif color == 'green' and cylinder_green_finish is False and cylinder_start:
        if start:
            print('执行green圆柱对应动作, 举起左手')
            cylinder_color = color
            cylinder_start = False
            get_cylinder = False
            #切换动作
            go_straight = 'left_hand_g'
            turn_left = 'left_hand_tl'
            turn_right = 'left_hand_tr'
            SSR.runActionGroup('left_hand')
        else:
            print('停止green圆柱对应动作\n')
            SSR.runActionGroup('stand_fast')
            cylinder_start = False
            #切换回动作
            go_straight = go_forward_fast
            turn_left = 'turn_left'
            turn_right = 'turn_right'
            cylinder_green_finish = True
    elif color == 'blue' and cylinder_blue_finish is False and cylinder_start:
        if start:
            print('执行blue圆柱对应动作，举起右手')
            cylinder_color = color
            cylinder_start = False
            get_cylinder = False
            go_straight = 'right_hand_g'
            turn_left = 'right_hand_tl'
            turn_right = 'right_hand_tr'
            SSR.runActionGroup('right_hand')
            time.sleep(0.1)
        else:
            print('停止blue圆柱对应动作\n')
            SSR.runActionGroup('stand_fast')
            cylinder_start = False
            go_straight = go_forward_fast
            turn_left = 'turn_left'
            turn_right = 'turn_right'
            cylinder_blue_finish = True
            
step = 1

count = 0
find_ball = False
get_bridge_color = False
get_yellow_line = False
get_yellow = False
ready_trace = True
trace = True
line_status = 0
find_goal = False
#寻迹，踢球逻辑动作处理部分，涉及到动作，所以单独作为一个线程运行，防止卡顿过长
def move():
    global get_black_line, black_line_centerx, set_black_line_centerx
    global go_straight, turn_left, turn_right, go_straight_kick
    global cross_line, get_cross_line, get_cylinder
    global cylinder_color, kick, ready_trace, trace
    global get_ball
    global X, Y
    global step, count 
    global find_ball
    global get_yellow_line
    global cylinder_start
    global line_status
    global find_goal
    global servo1_
    global min_value, max_value, min_value1, max_value1
    global get_bridge_color, bridge_up

    status = 1
    while True:
        if kick:
            if get_ball:
                count = 0
                if find_ball:
                    PWMServo.setServo(1, servo1 - 200, 100)
                    PWMServo.setServo(2, servo2, 100)
                    time.sleep(0.1)
                    find_ball = False
                    if status == 1:
                        get_ball = False
                        SSR.runActionGroup('turn_right')
                        get_ball = False
                        status = 5
                    elif status == 2:  
                        get_ball = False
                        SSR.runActionGroup('turn_left')
                        get_ball = False
                        status = 1
                    elif status == 3:
                        get_ball = False
                        SSR.runActionGroup('turn_left')
                        SSR.runActionGroup(go_straight_kick)
                        get_ball = False
                        status = 2
                    elif status == 4:
                        get_ball = False
                        SSR.runActionGroup('turn_right')
                        SSR.runActionGroup(go_straight_kick)
                        get_ball = False
                        status = 3

                elif step == 1:
                    print('进入步骤1')
                    if X - set_black_line_centerx> 50:#不在中心
                        print('X:' + str(X) + ' centerx:' + str(set_black_line_centerx) + '\n' + '球在右边-->>向右移动一步')
                        SSR.runActionGroup('turn_right')
                        time.sleep(0.2)
                    elif X - set_black_line_centerx < -50:#不在中心
                        print('X:' + str(X) + ' centerx:' + str(set_black_line_centerx) + '\n' + '球在左边-->>向左移动一步')     
                        SSR.runActionGroup('turn_left')
                        time.sleep(0.2)
                    else:#在中心
                        print('X:' + str(X) + ' centerx:' + str(set_black_line_centerx) + '\n' + '球在中心-->>转到步骤2')
                        step = 2
                elif step == 2:
                    print('进入步骤2')
                    if 60 < Y <= 120:
                        print('Y:' + str(Y) + '\n' + '球在最近处-->>转到步骤3')
                        step = 3
                    elif 20 < Y < 70:
                        print('Y:' + str(Y) + '\n' + '球在远处-->>向前走一步-->>回到步骤1')
                        SSR.runActionGroup('go_forward_one_step')
                        step = 1                         
                    elif 0 <= Y <= 20:
                        print('Y:' + str(Y) + '\n' + '球在远处-->>向前走-->>回到步骤1')
                        SSR.runActionGroup(go_forward)
                        step = 1 
                    else:
                        step = 1            
                elif step == 3:
                    print('进入步骤3')
                    if X - set_black_line_centerx > 40 or abs(X -  set_black_line_centerx) < 10:#不在中心，根据方向让机器人转向一步
                        print('X:' + str(X) + ' centerx:' + str(set_black_line_centerx) + '\n' + '球偏右-->>站立-->>右移一步')
                        #SSR.runActionGroup('stand_fast')
                        SSR.runActionGroup('right_move')
                    elif X - set_black_line_centerx < -40:
                        print('X:' + str(X) + ' centerx:' + str(set_black_line_centerx) + '\n' + '球偏左-->>站立-->>左移一步')
                        #SSR.runActionGroup('stand_fast')
                        SSR.runActionGroup('left_move')
                    else:
                        print('X:' + str(X) + ' centerx:' + str(set_black_line_centerx) + '\n' + '球在中心-->>转到步骤4')
                        step = 4                   
                elif step == 4:
                    print('进入步骤4')
                    if X - set_black_line_centerx >= 0:
                        print('X:' + str(X) + ' centerx:' + str(set_black_line_centerx) + '\n' + '球在右-->>右脚射门')
                        SSR.runActionGroup('right_shot_fast')                      
                    elif X - set_black_line_centerx < 0:
                        print('X:' + str(X) + ' centerx:' + str(set_black_line_centerx) + '\n' + '球在左-->>左脚射门')
                        SSR.runActionGroup('left_shot_fast')
                    step = 1
                    print('回到步骤1')
                get_ball = False
            else:
                if step != 3 and step != 4:
                    count += 1
                    if count >= 20:
                        count = 0
                        if not find_ball:                       
                            find_ball = True
                            get_ball = False
                        if status == 1:
                            status = 2
                            get_ball = False
                            PWMServo.setServo(1, servo1 - 150, 100)
                            PWMServo.setServo(2, servo2 + 300, 100)
                            time.sleep(0.1)
                        elif status == 2:
                            status = 3
                            get_ball = False
                            PWMServo.setServo(1, servo1 + 100, 100)
                            PWMServo.setServo(2, servo2 + 300, 100)
                            time.sleep(0.1)
                        elif status == 3:
                            status = 4
                            get_ball = False
                            PWMServo.setServo(1, servo1 + 100, 100)
                            PWMServo.setServo(2, servo2 - 300, 100)
                            time.sleep(0.1)
                        elif status == 4:
                            status = 1
                            get_ball = False
                            PWMServo.setServo(1, servo1 - 150, 100)
                            PWMServo.setServo(2, servo2 - 300, 100)
                            time.sleep(0.1)
                    else:
                        time.sleep(0.01)
                else:
                    time.sleep(0.01)
        else:
            if cross_line == 3 and bridge_up != 2:
                if bridge_up == 0:
                    #print(get_bridge_color, get_yellow_line, Y)
                    #只有识别到黄色且黄色处于图像的上部分，我们就切换为平稳步态，为了更稳定的识别黄色消失
                    if get_bridge_color and yellow_line_y2 >= Y > yellow_line_y1 and not get_yellow_line:
                        if get_yellow_line is False:
                            print('开始检测到黄线，切换步态')
                            get_yellow_line = True
                            go_straight = 'go_forward_one_step'
                            servo1_ = servo1 - 250
                            PWMServo.setServo(1, servo1_, 100)
                            time.sleep(0.1)
                    elif get_yellow_line and 120 >= Y > yellow_line_y:
                        #当检测不到黄色时，且此前检测到过黄色
                        print('开始执行上桥动作')
                        get_yellow_line = False
                        servo1_ = servo1 - 250
                        PWMServo.setServo(1, servo1_, 500)
                        SSR.runActionGroup('go_forward')
                        SSR.runActionGroup('shang')#上桥
                        print('上桥完成')
                        bridge_up = 1#标记一下上桥完成，防止下桥时识别成上桥
                        get_bridge_color = False
                else:
                    #当黄色处于下部分时
                    #print(get_bridge_color, Y)
                    if get_bridge_color and yellow_line_y3 < Y < yellow_line_y4:
                        #再次识别到黄色时，如果上桥标志为真，则执行下桥动作
                        servo1_ = servo1
                        PWMServo.setServo(1, servo1_, 500)                      
                        print('开始执行下桥动作')
                        SSR.runActionGroup('xia')#下桥
                        print('下桥完成')
                        bridge_up = 2
                        go_straight = go_forward_fast#步态切换回快
                        get_yellow_line = True
            if get_cross_line:
                if cross_line == 1:
                    SSR.runActionGroup(go_forward_fast)
                    get_black_line = False
                elif cross_line == 2 or cross_line == 4 or cross_line == 6:
                    trace = False
                    get_black_line = False
                    SSR.runActionGroup(go_forward_fast)
                    SSR.runActionGroup('stand_fast')
                    if cross_line == 6:
                        cylinder_start = True
                        if cylinder_blue_finish is False:
                            print('识别到blue圆柱')
                            action('blue', start=True)
                        elif cylinder_green_finish is False:
                            print('识别到green圆柱')
                            action('green', start=True)
                        elif cylinder_red_finish is False:
                            print('识别到red圆柱')
                            action('red', start=True)
                        trace = True
                    else:
                        PWMServo.setServo(1, servo1 + 300, 500)
                        PWMServo.setServo(2, servo2 - 600, 500)
                        time.sleep(0.5)
                        ready_trace = False
                    if cross_line == 6:
                        min_value = -1
                        max_value = 20000
                elif cross_line == 3 or cross_line == 5 or cross_line == 7:
                    cylinder_start = True
                    print('完全进入缓冲区')
                    action(cylinder_color, start=False)
                    SSR.runActionGroup(go_forward_fast)
                    if cross_line == 5:
                        SSR.runActionGroup(go_forward_fast)
                    if cross_line == 7:
                        min_value = 0
                        max_value = 22000
                        min_value1 = 5
                        servo1_ = servo1 - 200
                        PWMServo.setServo(1, servo1_, 500)
                        PWMServo.setServo(2, servo2, 500)
                        time.sleep(0.5)
                elif cross_line >= 9:
                    print('停止寻线')
                    trace = False
                    get_black_line = False
                    for i in range(4):
                        SSR.runActionGroup(go_forward_fast)
                    kick = True
                    print('开始执行踢球程序')
                get_cross_line = False
            if get_cylinder:
                ready_trace = True
                get_cylinder = False
                cylinder_start = True
                print('识别到' + cylinder_color + '圆柱')
                PWMServo.setServo(1, servo1, 500)
                PWMServo.setServo(2, servo2, 500)
                time.sleep(0.5)
                SSR.runActionGroup(go_forward)
                action(cylinder_color, start=True)
                trace = True
            elif get_black_line and trace:
                get_black_line = False
                #print(black_line_centerx, set_black_line_centerx)
                if line_status == 1:
                    PWMServo.setServo(1, servo1_, 100)
                    PWMServo.setServo(2, servo2, 100)
                    time.sleep(0.1)
                    SSR.runActionGroup(turn_left)
                    line_status = 0                     
                elif line_status == 2:
                    PWMServo.setServo(1, servo1_, 100)
                    PWMServo.setServo(2, servo2, 100)
                    time.sleep(0.1)
                    SSR.runActionGroup(turn_right)
                    line_status = 1
                elif line_status == 3:
                    PWMServo.setServo(1, servo1_, 100)
                    PWMServo.setServo(2, servo2, 100)
                    time.sleep(0.1)
                    line_status = 2
                elif abs(black_line_centerx - set_black_line_centerx) <= 15:
                    SSR.runActionGroup(go_straight)
                elif black_line_centerx - set_black_line_centerx > 15:
                    SSR.runActionGroup(turn_right)
                elif black_line_centerx - set_black_line_centerx < -15:
                    SSR.runActionGroup(turn_left)
                line_status = 0
            else:
                if get_black_line is False and trace:
                    for i in range((len(count_center) - 1), 0, -1):
                        if count_center[i] != 0:
                            if count_center[i] <= set_black_line_centerx:
                                SSR.runActionGroup(turn_left)#左转                                
                            elif count_center[i] > set_black_line_centerx:
                                SSR.runActionGroup(turn_right)#右转                   
#                     if line_status == 0:
#                         line_status = 1
#                         PWMServo.setServo(1, servo1_, 100)
#                         PWMServo.setServo(2, servo2 + 400, 300)
#                         time.sleep(0.8) 
#                     elif line_status == 1:
#                         line_status = 2
#                         PWMServo.setServo(1, servo1_, 100)
#                         PWMServo.setServo(2, servo2 - 400, 300)
#                         time.sleep(0.8)
#                     elif line_status == 2:
#                         line_status = 3
#                         PWMServo.setServo(1, servo1_, 100)
#                         PWMServo.setServo(2, servo2, 300)
#                         time.sleep(0.8)
                else:
                    time.sleep(0.01)

SSR.runActionGroup('0')    
threading.Thread(target=move, daemon=True).start()


while True:
    #获取一帧图像
    if orgFrame is not None:
        if Running:
            #对原图像不做任何处理，只作为显示，防止出错
            orgframe = orgFrame.copy()
            frame = orgFrame.copy()
            frame_width = frame.shape[:2][1]
            frame_height = frame.shape[:2][0]
            #如果到了踢球环节            
            if kick:
                color_identify(frame, target_color='blue')
            else:
                if ready_trace:
                    if trace:
                        Tracing(frame, frame_width, frame_height)
                else:
                    #检测圆柱颜色
                    cylinders_color = colors_identify(frame)
                    if cylinders_color is not None:
                        cylinder_color = cylinders_color
                        get_cylinder = True
                        
                #在第三个交叉路口后就是上下桥环节，所以此时需要开启识别黄色程序
                if cross_line == 3:
                    get_bridge_color = color_identify(frame)
            if debug == 1:#调试模式下
                cv2.namedWindow('orgframe', cv2.WINDOW_AUTOSIZE)#显示框命名
                cv2.imshow('orgframe', orgframe) #显示图像
                cv2.waitKey(1)
        else:
            time.sleep(0.01)
    else:
        time.sleep(0.01)
cv2.destroyAllWindows()
