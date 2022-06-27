import os
import time
import cv2

cmd_restart = "sudo systemctl restart mjpg_streamer@"
cmd_stop = "sudo systemctl stop mjpg_streamer@"

def CheckCamera():
    global cmd_restart, cmd_stop
    print('Checking Device...... \n')
    ret = os.popen("ls /dev/video*").read()
    video = ret.split('\n')[0].split('/')[-1]
    num = int(video[5:])
    print('Current device: ' + video + '\n')
    if num > 4:
        print('''No camera detected or wrong device!
1 Please make sure the camera is connected to the Raspberry Pi, then rerun this program
2 if you have already connected the camera, please unplug and plug it in again!''')
    else:   
        for i in range(4):
            if i != num:
                os.system(cmd_stop + 'video' + str(i))
        print('Restart Camera...... \n')
        os.system(cmd_restart + video)
        time.sleep(0.5)
# CheckCamera()
