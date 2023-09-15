'''
Sample Usage:-
python pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
'''


import numpy as np
import cv2
import sys
from which_aruco_dict import ARUCO_DICT
import argparse
import time


def draw(img, corners, imgpts):
    
    corner = tuple(corners[0].ravel()[:2])
    try:
        img = cv2.line(img, (round(corner[0]), round(corner[1])), (round(imgpts[0][0]), round(imgpts[0][1])), (255,0,0), 5)
        img = cv2.line(img, (round(corner[0]), round(corner[1])), (round(imgpts[1][0]), round(imgpts[1][1])), (0,255,0), 5)
        img = cv2.line(img, (round(corner[0]), round(corner[1])), (round(imgpts[2][0]), round(imgpts[2][1])), (0,0,255), 5)
    except IndexError:
        pass
        
    return img 

def pose_estimation(frame, matrix_coefficients, distortion_coefficients):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters_create()


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters) ## ADD CAMERA CALIBRATION
        #cameraMatrix=matrix_coefficients,
        #distCoeff=distortion_coefficients)

        # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                       distortion_coefficients)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            #cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec) 
            imgpts, jact = cv2.projectPoints(markerPoints, rvec, tvec, matrix_coefficients, distortion_coefficients)
            for idx in range(len(imgpts)):
                try:
                    frame = draw(frame,corners[idx], imgpts[idx])
                except IndexError:
                    pass    

    return frame

if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    
    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    #calibration_matrix_path = args["K_Matrix"]
    #distortion_coefficients_path = args["D_Coeff"]
    calibration_matrix_path = args["D:\Downloadsi\drone_control-main (2)\drone_control-main\calibration_matrix.npy"]
    distortion_coefficients_path = args["D:\Downloadsi\drone_control-main (2)\drone_control-main\distortion_coefficients.npy"]
    
    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    video = cv2.VideoCapture(0)
    time.sleep(2.0)

    while True:
        ret, frame = video.read()

        if not ret:
            break
        
        output = pose_estimation(frame, aruco_dict_type, k, d)

        cv2.imshow('Estimated Pose', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()