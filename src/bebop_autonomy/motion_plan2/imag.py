#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import time
import datetime
import numpy
import cv2
import cv2.aruco as aruco
import os
import pickle
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt

# plt.ion()
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

def restPovec(id):
    if id==9:
        povec=[.7112, 1.6383, 0, 1]
    elif id==11:
        povec=[.8001, 1.6383, 0, 1]
    elif id==13:
        povec=[.7112, 1.5113, 0, 1]
    elif id==15:
        povec=[.8001, 1.5113, 0, 1]
    elif id==1:
        povec=[.7112, 1.9304, 0, 1]
    elif id==3:
        povec=[.8001, 1.9304, 0, 1]
    elif id==5:
        povec=[.7112, 1.8034, 0, 1]
    elif id==7:
        povec=[.8001, 1.8034, 0, 1]
    elif id==17:
        povec=[1.335, 1.9304, 0, 1]
    elif id==19:
        povec=[1.4351, 1.9304, 0, 1]
    elif id==21:
        povec=[1.3335, 1.8034, 0, 1]
    elif id==23:
        povec=[1.4351, 1.8034, 0, 1]
    elif id==25:
        povec=[1.335, 1.6383, 0, 1]
    elif id==27:
        povec=[1.4351, 1.6383, 0, 1]
    elif id==29:
        povec=[1.3335, 1.5113, 0, 1]
    elif id==31:
        povec=[1.4351, 1.5113, 0, 1]
    else:
        povec=[0,0,0,0]
    return povec

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


# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    start_time = time.time()
    print("Received an image!")
    global i
    if i%5==0:
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # cv2.imshow("img",cv2_img)
            # cv2.waitKey(1)
            # cv2.imwrite('images/'+str(i)+'.jpeg', cv2_img)
            # elaspsed_time = time.time()-start_time
            # print(elaspsed_time)

            # ___________________________
            # Capturing each frame of our video stream
            QueryImg = cv2_img
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
                    #    QreturnpovecueryImg = aruco.drawAxis(QueryImg, cameraMatrix, distCoeffs, rvec, tvec, 0.3)
                    # Estimate the posture per each Aruco marker
                    rvecs, tvecs,_ = aruco.estimatePoseSingleMarkers(corners, 0.07, cameraMatrix, distCoeffs)
                    i=0
                    arucopos=numpy.zeros((len(ids)+1,4))
                    for rvec, tvec in zip(rvecs, tvecs):
                        povec=restPovec(ids[i])
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
                        arucopos[i,:]=povec
                        i=i+1
                        if i==1:

                            posavg=cameracord
                        elif i==len(ids):
                            posavg = cameracord+posavg
                            posavg = posavg/i  #output will hve homogenous plane coordinate =2, because cameracord=arucocord+povec, and we add two  ones
                            print(posavg)
                            arucopos[i,:]=posavg
                            # print(arucopos)
                            x=arucopos[:,0]
                            y=arucopos[:,1]
                            z=arucopos[:,2]
                            #
                            # # print(x,y,z)
                            # ax.scatter(x, y, z, c='r', marker='o')
                            # ax.set_zlim([0, 1])
                            # ax.set_xlabel('X Label')
                            # ax.set_ylabel('Y Label')
                            # ax.set_zlabel('Z Label')
                            # plt.draw()
                            # plt.pause(0.002)
                            # ax.cla()


                        else:
                            posavg = cameracord+posavg

                    i=0
                # Display our image
                # cv2.imshow('QueryImage', QueryImg)
        # # Display our image
        cv2.imshow('QueryImage', cv2_img)
        # # cv2.imwrite('det/det_'+str(i)+".png",QueryImg)
        cv2.waitKey(1)
        print("computation time",time.time()-start_time)



def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/bebop/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback,  queue_size = 1)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    i=0
    main()
