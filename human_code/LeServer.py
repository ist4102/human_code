#!/usr/bin/python3
# encoding: utf-8

import socketserver
import threading
import time
import PWMServo
import re
from config import *
from LeCmd import LeError
import LeCmd
import sys
import Serial_Servo_Running
import RPi.GPIO as GPIO

class ServoServer(socketserver.BaseRequestHandler):
    Flag = True

    def handle(self):
        print("已连接")
        conn = self.request
        recv = b''
        recv_data = ""
        while self.Flag:
            try:
                recv = conn.recv(1024)
                recv_data = recv.decode()
                #print('recv_data', recv_data)
                #print('***********')
                if not recv_data:
                    self.Flag = False
                    print("break")
                    break
                rdata = recv_data.split("\r\n")  # 分割               
                rex = re.compile(r'^(I[0-9]{3}).*')  # 判断收到的指令是否符合规则
                data = rdata[0]
                match = data
                match = rex.match(match)
                if match:
                    if not 0 == match.start() or not len(data) == match.end():
                        print("错误指令 1")
                    else:
                        print ('data', data)
                        data = data.split('-')
                        cmd = data[0][1:5]
                        del data[0]
                        par = []
                        try:
                            cmd = int(cmd)
                            # print ('cmd', cmd)
                            if 3 <= cmd <= 9:
                                # print(rdata)
                                LeCmd.cmd_list[cmd](conn, data)
                            else:
                                for p in data:
                                    par.append(int(p))
                                # print ('par', par)
                                LeCmd.cmd_list[cmd](par)
                        except LeError as err:
                            print(err.msg)
                            print(err.data)
                else:
                    #print(data)
                    print("错误指令 2")
                if not self.Flag:
                    print("break1")
                    break
            except Exception as e:
                print(e)
                break

    def finish(self):
        print("已断开")


class LeServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True

if __name__ == "__main__":
    buzzer_pin = 23
    GPIO.setup(buzzer_pin, GPIO.OUT)
    GPIO.output(buzzer_pin, 1)

    PWMServo.setServo(1, 1500, 500)
    PWMServo.setServo(2, 1500, 500)
    LeCmd.cmd_i001([500, 16, 1, 500, 2, 388, 3, 500, 4, 594, 5, 500, 6, 575, 7, 800, 8, 725, 9, 500, 10, 612, 11, 500,
                    12, 406, 13, 500, 14, 425, 15, 200, 16, 275])    # 立正
    time.sleep(0.5)
    GPIO.output(buzzer_pin, 0)
    server = LeServer(("", 1075), ServoServer)
    try:
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.start()
        while True:
            time.sleep(0.1)
    except:
        server.shutdown()
        server.server_close()
