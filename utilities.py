from djitellopy import Tello
import cv2, math, time
import threading
import datetime
import os
from os import path
from cv2 import aruco
import matplotlib as mpl
import pandas as pd
import numpy as np
from PIL import Image
from PIL import ImageTk
import tkinter as tki
from tkinter import Toplevel, Scale

 
def initializeTello():
    myDrone = Tello()
    myDrone.connect()
    myDrone.for_back_velocity = 0
    myDrone. left_right_velocity = 0
    myDrone.up_down_velocity = 0
    myDrone.yaw_velocity = 0
    myDrone.speed = 0
    print(myDrone.get_battery())
    myDrone.streamoff()
    myDrone.streamon()
    return myDrone
 


def control(kp, ki, kd, error):
    error_int += error
    if error_prev is None:
        error_prev = error
    error_deriv = error - error_prev
    error_prev = error
    return kp*error + ki*error_int + kd*error_deriv



def findAruco(frame, arucoId, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #cv2.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    #parameters = cv2.aruco.DetectorParameters_create()

    cv2.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, _  = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters) ## ADD CAMERA CALIBRATION
   
    #frame_markers = aruco.drawDetectedMarkers(self.frameCopy, corners, ids)
        
        
    if ids is not None:
        for i in range(len(ids)):
            if ids[i] == arucoId:
                c = corners[i]
                rvec, tvec,_ = aruco.estimatePoseSingleMarkers(c, 0.10, matrix_coefficients, distortion_coefficients)
                    
                if rvec is not None and tvec is not None:
                    #cv2.drawFrameAxes(self.frameCopy, self.cameraMatrix, self.distCoeffs, rvec, tvec, 0.20)

                    # Convert to Euler
                    R_mat = np.matrix(cv2.Rodrigues(rvec)[0])
                    roll, pitch, yaw = rotationMatrixToEulerAngles(R_mat)
                    rollD = math.degrees(roll)
                    pitchD = math.degrees(pitch) 
                    yawD = math.degrees(yaw)
                    """
                        print('To je tvec: ',tvec)
                        print 'Roll %d' % rollD
                        print 'Pitch %d' % pitchD
                        print 'Yaw %d' % yawD
                    """    
                    rot = np.array([rollD, pitchD, yawD])
                    return np.array([tvec[0][0], yawD])

    return None


def trackAruco(myDrone,pid,pError,ierror, target, target_y):

    ## PID
    speed = [0, 0, 0, 0]
    error = [target[0], target[1], target[2]*100-myDrone.get_height(), target_y]
    
    ierror[0] += error[0]
    ierror[1] += error[1]
    ierror[2] += error[2]
    ierror[3] += error[3]
    for i in range(0,4):
        heh = pid[0]*error[i] + pid[1]*(error[i]-pError[i]) + pid[2]*ierror[i]
        speed[i] = int(np.clip(heh,-100,100))
 
 
    print(speed)
    myDrone.for_back_velocity = speed[0]
    myDrone.left_right_velocity = speed[1]
    myDrone.up_down_velocity = speed[2]
    myDrone.yaw_velocity = speed[3]
    if myDrone.send_rc_control:
        myDrone.send_rc_control(myDrone.left_right_velocity,
                                myDrone.for_back_velocity,
                                myDrone.up_down_velocity,
                                myDrone.yaw_velocity)
    return error, ierror

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def controlAll(T, R):
    currTime = time.time()
    #print("Control",currTime)

def onClose(self):
    print("[INFO] closing...")
    self.stopEvent.set()
    del self.tello
    self.root.quit()