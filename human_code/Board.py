#!/usr/bin/env python3
import os
import sys
import time
import pigpio
import RPi.GPIO as GPIO
from BusServoCmd import *

#幻尔科技TonyPi扩展板sdk#
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

def setBusServoID(oldid, newid):
    """
    配置舵机id号, 出厂默认为1
    :param oldid: 原来的id， 出厂默认为1
    :param newid: 新的id
    """
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_ID_WRITE, newid)

def getBusServoID(id=None):
    """
    读取串口舵机id
    :param id: 默认为空
    :return: 返回舵机id
    """
    
    while True:
        if id is None:  # 总线上只能有一个舵机
            serial_servo_read_cmd(0xfe, LOBOT_SERVO_ID_READ)
        else:
            serial_servo_read_cmd(id, LOBOT_SERVO_ID_READ)
        # 获取内容
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ID_READ)
        if msg is not None:
            return msg

def setBusServoPulse(id, pulse, use_time):
    """
    驱动串口舵机转到指定位置
    :param id: 要驱动的舵机id
    :pulse: 位置
    :use_time: 转动需要的时间
    """

    pulse = 0 if pulse < 0 else pulse
    pulse = 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_TIME_WRITE, pulse, use_time)

def stopBusServo(id=None):
    '''
    停止舵机运行
    :param id:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_STOP)

def setBusServoDeviation(id, d=0):
    """
    调整偏差
    :param id: 舵机id
    :param d:  偏差
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_ADJUST, d)

def saveBusServoDeviation(id):
    """
    配置偏差，掉电保护
    :param id: 舵机id
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_WRITE)

def getBusServoDeviation(id):
    '''
    读取偏差值
    :param id: 舵机号
    :return:
    '''
    # 发送读取偏差指令
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_READ)
        # 获取
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_OFFSET_READ)
        if msg is not None:
            return msg

def setBusServoAngleLimit(id, low, high):
    '''
    设置舵机转动范围
    :param id:
    :param low:
    :param high:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_WRITE, low, high)

def getBusServoAngleLimit(id):
    '''
    读取舵机转动范围
    :param id:
    :return: 返回元祖 0： 低位  1： 高位
    '''
    
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_LIMIT_READ)
        if msg is not None:
            count = 0
            return msg

def setBusServoVinLimit(id, low, high):
    '''
    设置舵机电压范围
    :param id:
    :param low:
    :param high:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_VIN_LIMIT_WRITE, low, high)

def getBusServoVinLimit(id):
    '''
    读取舵机转动范围
    :param id:
    :return: 返回元祖 0： 低位  1： 高位
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_LIMIT_READ)
        if msg is not None:
            return msg

def setBusServoMaxTemp(id, m_temp):
    '''
    设置舵机最高温度报警
    :param id:
    :param m_temp:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, m_temp)

def getBusServoTempLimit(id):
    '''
    读取舵机温度报警范围
    :param id:
    :return:
    '''
    
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        if msg is not None:
            return msg

def getBusServoPulse(id):
    '''
    读取舵机当前位置
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_POS_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_POS_READ)
        if msg is not None:
            return msg

def getBusServoTemp(id):
    '''
    读取舵机温度
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_READ)
        if msg is not None:
            return msg

def getBusServoVin(id):
    '''
    读取舵机电压
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_READ)
        if msg is not None:
            return msg

def restBusServoPulse(oldid):
    # 舵机清零偏差和P值中位（500）
    serial_servo_set_deviation(oldid, 0)    # 清零偏差
    time.sleep(0.1)
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_MOVE_TIME_WRITE, 500, 100)    # 中位

##掉电
def unloadBusServo(id):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 0)

##读取是否掉电
def getBusServoLoadStatus(id):
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        if msg is not None:
            return msg

