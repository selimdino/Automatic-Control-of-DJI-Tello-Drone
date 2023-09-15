from djitellopy import tello
from threading import Thread
import pose_estimation
import cv2
import numpy as np
import time
import aruco
import matplotlib.pyplot as plt
from utilities2 import *
import math
import os
from queue import Queue

arucoId = 1
t = tello.Tello()
c_matx = np.load("D:\Downloadsi\drone_control-main (2)\drone_control-main\calibration_matrix.npy")
d_coeff = np.load("D:\Downloadsi\drone_control-main (2)\drone_control-main\distortion_coefficients.npy")
img_queue = Queue()
target_queue = Queue()


def get_video_feed():
    use_tello_cam = True
    if use_tello_cam:
        t.connect()
        t.streamoff()
        print(t.get_battery())
        t.streamon()
    else:
        cap = cv2.VideoCapture(0)
    cv2.namedWindow("drone")

    while True:
        if not use_tello_cam:
            ret, img = cap.read()
        else:
            img = t.get_frame_read().frame
        c_matx = np.load("D:\Downloadsi\drone_control-main (2)\drone_control-main\calibration_matrix.npy")
        d_coeff = np.load("D:\Downloadsi\drone_control-main (2)\drone_control-main\distortion_coefficients.npy")
        img = pose_estimation.pose_estimation(img, c_matx, d_coeff)
        # put the image in the queue
        img_queue.put(img)
        cv2.imshow("drone", img)
        #cv2.waitKey(1)            #ti si nam falio
        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            break
     #  plt.show()

def aruco_finding(c_matx, d_coeff):
    while True:
        img = img_queue.get()
        target = findAruco(img, arucoId, c_matx, d_coeff)
          # add a 0.1 second waiting time
#       print(target)
        target_queue.put(target)
    
def perform_aruco_tracking(c_matx, d_coeff):
    #False means use webcam feed, true means use Tello camera feed
    startCounter = 1  # for no Flight 1   - for flight 0    
    prev_error =  [0., 0., 0., 0.]
    int_error = [0., 0., 0., 0.] 
    pid = [0.5, 0.01, 0.01]

    t.for_back_velocity = 0
    t.left_right_velocity = 0
    t.up_down_velocity = 0
    t.yaw_velocity = 0
    t.speed = 0
    while True:
        # code for performing ArUco tracking
        if startCounter == 0:
            t.takeoff()
            startCounter = 1
        
        arucoId = 1 #stavit cemo da je prvi jedan
        while (arucoId != 0):
            target = target_queue.get()
            while target is None:
                print('target is none')
                #t.send_rc_control(0,0,20,0)
                #t.sleep(2)
                t.send_rc_control(0,0,0,0)
     #          t.send_rc_control(0,0,0,15)
                target = target_queue.get()
                if target is not None:
                    break
                #t.move_down(45)
     #          t.send_rc_control(0,0,0,-15)
                 
            while arucoId == 1 or arucoId == 2 or arucoId == 4 or arucoId == 5 or arucoId == 6:
                target = target_queue.get()
             #   target_pos = np.asarray(target[0])
             #   target_yaw = target[1]
                                
                first_element = target[0]  # Get the first element of the list

                print(first_element)  # Output: -0.02217616
             #   target_x = target_pos[0]

            #   print(target) 
            #    first_element = target_pos[0]
            
           #     print(first_element)
                print(prev_error)
                '''
                prev_error, int_error = trackAruco(t, pid, prev_error,int_error,target_pos,target_yaw)

                if prev_error[0] < 0.2:
                    print("Manjše od 0.2")
                    t.send_rc_control(30,0,0,0)
                    time.sleep(4)
                    if arucoId == 6:
                        arucoId = 0
                        prev_error = [0.,0.,0.,0.]
                        int_error = [0.,0.,0.,0.]
                        break
                    else:
                        arucoId += 1
                        prev_error = [0.,0.,0.,0.]
                        int_error = [0.,0.,0.,0.]
                        break
                
            while arucoId == 3:
                prev_error, int_error = trackAruco(t, pid, prev_error,int_error,target_pos,target_yaw)
                target = target_queue.get()
                target_pos = target[0]
                target_yaw = target[1]
                if prev_error[0] < 0.2:
                    t.send_rc_control(30,0,0,0)
                    time.sleep(4)
                    t.send_rt_control(0,0,0,0)
                    time.sleep(2)
                    t.rotate_clockwise(180)
                    t.move_left(100)  #mozda treba promijeniti vrijednost
                    arucoId += 1
                    prev_error = [0.,0.,0.,0.]
                    int_error = [0.,0.,0.,0.]
                    break

            if arucoId == 0:
                prev_error[2] == -0.2
            while arucoId == 0:
                target = target_queue.get()
                target_pos = target[0]
                target_yaw = target[1]
                prev_error, int_error = trackAruco(t, pid, prev_error,int_error,target_pos,target_yaw)
                if prev_error[0] < 0.15 and prev_error[1] < 0.15:
                    t.land()
                    break

            prev_error = [0.,0.,0.,0.]
            int_error = [0.,0.,0.,0.]
    
            if cv2.waitKey(1) & 0xFF == ord('q'):
                t.land()
                break
            '''

video_feed_thread = Thread(target=get_video_feed)
video_feed_thread.start()

aruco_find_thread = Thread(target=aruco_finding, args = (c_matx, d_coeff))
aruco_find_thread.start()

aruco_track_thread = Thread(target=perform_aruco_tracking, args = (c_matx, d_coeff))
aruco_track_thread.start()
