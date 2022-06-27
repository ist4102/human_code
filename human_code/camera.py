#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
#2020/2/27/Aiden
import os
import cv2
import sys
import time
import threading

#usb摄像头获取画面
class USBCamera():
    
    #初始化
    def __init__(self, resolution = None):
        self.orgFrame = None
        self.Running = True
        #如果摄像头正常则开启，否则抛出异常提示，退出程序
        self.cameraVideo = self.checkcamera()
        if self.cameraVideo is not None:
            self.cap = cv2.VideoCapture(self.cameraVideo)
            #如果分辨率有设置
            if resolution is not None:
                #将分辨率取整，取正
                self.resolution = (abs(int(resolution[0])), abs(int(resolution[1])))
                #如果分辨率不在32～1920之间，打印提示信息，退出程序
                if 1920 < self.resolution[0] or self.resolution[0] < 32 or 1920 < self.resolution[1] or self.resolution[1] < 32:               
                    print('Wrong resolution, resolution should be between 32 and 1920')
                    sys.exit(0)
            #如果没有设置分辨率，则自动获取摄像头的分辨率
            else:
                self.resolution = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            print('USBCamera loaded... warming camera\n')
            #开启摄像头画面更新子线程
            t = threading.Thread(target=self.update)
            t.setDaemon(True)
            t.start()
        #如果没有检测到摄像头，打印提示信息，退出程序
        else:
            print('''No camera detected or wrong device!
        1 Please make sure the camera is connected to the Raspberry Pi, then rerun this program
        2 if you have already connected the camera, please unplug and plug it in again!''')
            sys.exit(0)        
                        
    #获取摄像头的设备驱动
    def checkcamera(self):
        print('Checking Device...... \n')
        
        #获取摄像头驱动信息
        ret = os.popen("ls /dev/video*").read()
        video = ret.split('\n')[0].split('/')[-1]
        num = int(video[5:])
        
        #打印检测到的摄像头驱动
        print('Current device: ' + video + '\n')
        
        #如果设备驱动序号大于4就认为摄像头不存在，否则返回设备号
        if num > 4:
            return None
        else:   
            return num
    
    #返回一帧图片
    def getframe(self):
        #如果图片不为空则返回图片
        if self.orgFrame is not None:
            return self.orgFrame
    
    #画面刷新
    def update(self):
        while True:
            if self.Running is True:
                if self.cap.isOpened():
                    #读取图片
                    ret, orgframe = self.cap.read()          
                    if ret:
                        #将摄像头画面缩小以便处理
                        self.orgFrame = cv2.resize(orgframe, (self.resolution[0],self.resolution[1]), interpolation = cv2.INTER_CUBIC)
                    else:
                        time.sleep(0.01)
                else:
                    time.sleep(0.01)
            else:
                break
    
    #关闭摄像头
    def shutdown(self):
        print('stoping USBCamera\n')
        #释放摄像头，以便下次使用
        self.cap.release()
        cv2.destroyAllWindows()
        self.Running = False

#使用范例
if __name__ == '__main__':
    ##鼠标左键双击退出
    Running = True
    def closeEvent(event,x,y,flags,param):
        global Running 
        if event==cv2.EVENT_LBUTTONDBLCLK:
            Running = False

    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame', closeEvent)

    MyCamera = USBCamera((640, 480))
    while True:
        frame = MyCamera.getframe()
        if frame is not None:
            cv2.putText(frame, 'Enter esc to quit!', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow('frame', frame)
            key = cv2.waitKey(1)
            if key == 27 or Running is False:
                break            
    MyCamera.shutdown()
