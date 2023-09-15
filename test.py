from djitellopy import tello
import pose_estimation
import cv2
import numpy as np
import time
import aruco
import matplotlib.pyplot as plt
import numpy as np
from utilities import *
#False means use webcam feed, true means use Tello camera feed
use_tello_cam = True 
#startCounter = 1  # for no Flight 1   - for flight 0


prev_error =  [0., 0., 0., 0.]
int_error = [0., 0., 0., 0.] 
pid = [0.5, 0.01, 0.01]
arucoId = 1


if use_tello_cam:
    t = tello.Tello()
    t.connect()
    t.streamoff()
    print(t.get_battery())
    t.streamon()
else:
    cap = cv2.VideoCapture(0)
cv2.namedWindow("drone")
prev_time = 0
i=0

t.for_back_velocity = 0
t.left_right_velocity = 0
t.up_down_velocity = 0
t.yaw_velocity = 0

#if startCounter == 0:
#    t.takeoff()
#    startCounter = 1

while True:
    if not use_tello_cam:
        ret, img = cap.read()
    else:
        img = t.get_frame_read().frame
        #img = cv2.resize(img, (360, 240))
        #print(prev_time, time.time())
        #if time.time() - prev_time > 4:
        #    plt.imsave("Calibration/data/img_{}.png".format(i), img)
        #    prev_time = time.time()
        #    i+=1
        
    c_matx = np.load("D:\Downloadsi\drone_control-main (2)\drone_control-main\calibration_matrix.npy")
    d_coeff = np.load("D:\Downloadsi\drone_control-main (2)\drone_control-main\distortion_coefficients.npy")
    img = pose_estimation.pose_estimation(img, c_matx, d_coeff)
    cv2.imshow("drone", img)
    plt.show()

    
    target = findAruco(img, arucoId, c_matx, d_coeff)
    target_pos = target[0]
    target_yaw = target[1]
    prev_error, int_error = trackAruco(t, pid, prev_error,int_error,target_pos,target_yaw)

    if i > 20 or cv2.waitKey(1) & 0xFF == ord('q'):
        break