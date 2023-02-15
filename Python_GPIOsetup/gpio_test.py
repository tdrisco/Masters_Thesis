import RPi.GPIO as GPIO
import time

vicon = 26
motionCap_flag = False
capFinished = False

GPIO.setmode(GPIO.BCM)
GPIO.setup(vicon, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def vicon_end(vicon):
    global motionCap_flag
    if motionCap_flag == False:
        motionCap_flag = True
        print("Motion Capture Started-----------------")
    else:
        motionCap_flag = False
        print("Motion Capture Ended--------------------")
        capFinshed = True

GPIO.add_event_detect(vicon, GPIO.BOTH, callback=vicon_end)

# try:
#     while True:
#         print('Vicon status = ', GPIO.input(vicon))        
# except KeyboardInterrupt:
#     print("Data Stream Interrupted")
#     pass

try:
    while True:
        if motionCap_flag:
           print("Recording...")
       # else:
           # print("OFF")
        #print("Capture Status {}".format(motionCap_flag))        
except capFinished:
    print("Done")
    pass


GPIO.cleanup()