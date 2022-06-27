#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import threading
import sqlite3 as sql
from hwax import HWAX
import BusServoCmd as BSC
from Board import setBusServoPulse

runningAction = False
stop_action = False
stop_action_group = False

def stopAction():
    global stop_action
    
    stop_action = True

def stopActionGroup():
    global stop_action_group
    
    stop_action_group = True 

__end = False
__start = True
current_status = ''
def runActionGroup(actName, times=1, with_stand=False): 
    global __end
    global __start
    global current_status
    global stop_action_group
    
    temp = times
    while True:
        if temp != 0:
            times -= 1
        try:
            if (actName != 'go_forward' and actName != 'go_forward_fast' and actName != 'back' and actName != 'back_fast') or stop_action_group:
                if __end:
                    __end = False
                    if current_status == 'go':
                        runAction('go_forward_end')
                    else:
                        runAction('back_end')
                    #print('end2')
                if stop_action_group:
                    __end = False
                    __start = True
                    stop_action_group = False                        
                    #print('stop_action_group')
                    break
                __start = True
                if times < 0:
                    __end = False
                    __start = True
                    stop_action_group = False 
                    break
                runAction(actName)
            else:
                if times < 0:
                    #print('end1')
                    if with_stand:
                        if actName == 'go_forward' or actName == 'go_forward_fast':
                            runAction('go_forward_end')
                        else:
                            runAction('back_end')
                    break
                if __start:
                    __start = False
                    __end = True
                    #print('start')
                    if actName == 'go_forward':                       
                        runAction('go_forward_start')
                        current_status = 'go'
                    elif actName == 'go_forward_fast':
                        runAction('go_forward_start_fast')
                        current_status = 'go'
                    elif actName == 'back':
                        runAction('back_start')
                        runAction('back')
                        current_status = 'back'                    
                    elif actName == 'back_fast':
                        runAction('back_start')
                        runAction('back_fast')
                        current_status = 'back'
                else:
                    runAction(actName)
        except BaseException as e:
            print(e)

def runAction(actNum):
    '''
    运行动作组，无法发送stop停止信号
    :param actNum: 动作组名字 ， 字符串类型
    :param times:  运行次数
    :return:
    '''
    global runningAction
    global stop_action
    
    if actNum is None:
        return

    hwaxNum = "/home/pi/human_code/ActionGroups/" + actNum + ".hwax"
    actNum = "/home/pi/human_code/ActionGroups/" + actNum + ".d6a"

    if os.path.exists(hwaxNum) is True:
        if runningAction is False:
            runningAction = True
            BSC.portWrite()
            hwax = HWAX(hwaxNum, BSC.serialHandle)
            hwax.reset()
            while True:
                if stop_action is True:
                    stop_action = False
                    print('stop')                   
                    break
                ret = hwax.next()
                if ret is None:
                    hwax.reset()
                    break
            hwax.close()
            runningAction = False

    elif os.path.exists(actNum) is True:
        if runningAction is False:
            runningAction = True
            ag = sql.connect(actNum)
            cu = ag.cursor()
            cu.execute("select * from ActionGroup")
            while True:
                act = cu.fetchone()
                if stop_action is True:
                    stop_action = False
                    print('stop')                    
                    break
                if act is not None:
                    for i in range(0, len(act) - 2, 1):
                        setBusServoPulse(i + 1, act[2 + i], act[1])
                    time.sleep(float(act[1])/1000.0)
                else:   # 运行完才退出
                    break
            runningAction = False
            
            cu.close()
            ag.close()
    else:
        runningAction = False
        print("未能找到动作组文件")
    
if __name__ == '__main__':
    runActionGroup('go_forward', 1)
