# -*- coding: utf-8 -*-
"""
Created on Wed Jul 17 18:46:30 2019

@author: yadityavarma
"""

# The following code is used to watch a video stream, detect Aruco markers, and use
# a set of markers to determine the posture of the camera in relation to the plane
# of markers.
#
# Assumes that all markers are on the same plane, for example on the same piece of paper
#
# Requires camera calibration (see the rest of the project for example calibration)


import numpy
import cv2
import cv2.aruco as aruco
import os
import pickle
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# Check for camera calibration data

if not os.path.exists('./calibration.pckl'):
    print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
    exit()
else:
    f = open('calibration.pckl', 'rb')
    (cameraMatrix, distCoeffs, _, _) = pickle.load(f)
    f.close()
    if cameraMatrix is None or distCoeffs is None:
        print("Calibration issue. Remove ./calibration.pckl and recalibrate your camera with CalibrateCamera.py.")
        exit()

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_1000)

# Create vectors we'll be using for rotations and translations for postures
rvecs, tvecs = None, None

for g in range(1,11):
    # Capturing each frame of our video stream
    QueryImg = cv2.imread(str(g)+".jpeg")
    ret = True
    if ret == True:
        # grayscale image
        gray = cv2.cvtColor(QueryImg, cv2.COLOR_BGR2GRAY)

        # Detect Aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

        # Refine detected markers
        # Eliminates markers not part of our board, adds missing markers to the board

        #Outline all of the markers detected in our image
        QueryImg = aruco.drawDetectedMarkers(QueryImg, corners, borderColor=(0, 0, 255))
        camcord=numpy.zeros((3,6))

        # Require 15 markers before drawing axis    i=i+1
        if ids is not None and len(ids) > 0:
            # Estimate the posture of the gridboard, which is a construction of 3Dtransformation=numpy.append(temp,tvec,axis=1) space based on the 2D video
            #pose, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs)
            #if pose:
            #    # Draw the camera posture calculated from the gridboard
            #    QueryImg = aruco.drawAxis(QueryImg, cameraMatrix, distCoeffs, rvec, tvec, 0.3)
            # Estimate the posture per each Aruco marker
            rvecs, tvecs,_objpoints = aruco.estimatePoseSingleMarkers(corners, 0.07, cameraMatrix, distCoeffs)
            i=0
            for rvec, tvec in zip(rvecs, tvecs):
                if ids[i]==9:
                    povec=[.7112, 1.6383, 0, 1]
                elif ids[i]==11:
                    povec=[.8001, 1.6383, 0, 1]
                elif ids[i]==13:
                    povec=[.7112, 1.5113, 0, 1]
                elif ids[i]==15:
                    povec=[.8001, 1.5113, 0, 1]
                elif ids[i]==1:
                    povec=[.7112, 1.9304, 0, 1]
                elif ids[i]==3:
                    povec=[.8001, 1.9304, 0, 1]
                elif ids[i]==5:
                    povec=[.7112, 1.8034, 0, 1]
                elif ids[i]==7:
                    povec=[.8001, 1.8034, 0, 1]
                elif ids[i]==17:
                    povec=[1.335, 1.9304, 0, 1]
                elif ids[i]==19:
                    povec=[1.4351, 1.9304, 0, 1]
                elif ids[i]==21:
                    povec=[1.3335, 1.8034, 0, 1]
                elif ids[i]==23:
                    povec=[1.4351, 1.8034, 0, 1]
                elif ids[i]==25:
                    povec=[1.335, 1.6383, 0, 1]
                elif ids[i]==27:
                    povec=[1.4351, 1.6383, 0, 1]
                elif ids[i]==29:
                    povec=[1.3335, 1.5113, 0, 1]
                elif ids[i]==31:
                    povec=[1.4351, 1.5113, 0, 1]
                # else:
                #     povec=[0,0,0,1]
                QueryImg = aruco.drawAxis(QueryImg, cameraMatrix, distCoeffs, rvec, tvec, 0.1)
                cv2.imwrite("1.png",QueryImg)
                Rt,_Jacobian=cv2.Rodrigues(rvec)
                tvec=numpy.transpose(tvec[0])
                tvec=tvec.reshape(3,1)
                temp=numpy.concatenate(Rt)
                temp=temp.reshape(3,3)
                transformation=numpy.append(temp,tvec,axis=1)
                append=numpy.array([0,0,0,1]).reshape(1,4)
                transformation=numpy.append(transformation,append,axis=0)
                transformation=numpy.linalg.inv(transformation)
                arucocord=numpy.matmul(transformation,numpy.array([0,0,0,1]))
                cameracord=arucocord+povec
                # print(cameracord)

                i=i+1
                if i==1:
                    posavg=cameracord
                elif i==len(ids):
                    posavg = cameracord+posavg
                    posavg = posavg/i
                    print(posavg)     #output will hve homogenous plane coordinate =2, because cameracord=arucocord+povec, and we add two  ones
                else:
                    posavg = cameracord+posavg

            i=0
        # Display our image
        cv2.imshow('QueryImage', QueryImg)

    # Exit at the end of the video on the 'q' keypress
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
