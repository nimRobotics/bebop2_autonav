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
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# Instantiate CvBridge
bridge = CvBridge()
def image_callback(msg):
    start_time = time.time()
    print("Received an image!")
    global i
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # ts = time.time()
        # st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H-%M-%S')
        i=i+1
        print(i)
        # cv2.imshow("img",cv2_img)
        # cv2.waitKey(1)
        # cv2.imwrite('images/'+str(i)+'.jpeg', cv2_img)
        elaspsed_time = time.time()-start_time
        print(elaspsed_time)

        ################################################## marker detect starts
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
        # grayscale image
        gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        QueryImg = cv2_img

        # Detect Aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

        # Refine detected markers
        # Eliminates markers not part of our board, adds missing markers to the board

        #Outline all of the markers detected in our image
        QueryImg = aruco.drawDetectedMarkers(QueryImg, corners, borderColor=(0, 0, 255))
        camcord=numpy.zeros((3,6))

        # Require 15 markers before drawing axis
        if ids is not None and len(ids) > 0:
            # Estimate the posture of the gridboard, which is a construction of 3D space based on the 2D video
            #pose, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs)
            #if pose:
            #    # Draw the camera posture calculated from the gridboard
            #    QueryImg = aruco.drawAxis(QueryImg, cameraMatrix, distCoeffs, rvec, tvec, 0.3)
            # Estimate the posture per each Aruco marker
            rvecs, tvecs,_objpoints = aruco.estimatePoseSingleMarkers(corners, 0.19, cameraMatrix, distCoeffs)
            objpoints=numpy.concatenate(_objpoints)
            objpoints=objpoints.reshape(4,3)
            append=numpy.array([1,1,1,1]).reshape(4,1)
            objpoints=numpy.append(objpoints,append,axis=1)
            append=numpy.array([0,0,0,1]).reshape(1,4)
            objpoints=numpy.append(objpoints,append,axis=0)

            o=0
            for rvec, tvec in zip(rvecs, tvecs):
                QueryImg = aruco.drawAxis(QueryImg, cameraMatrix, distCoeffs, rvec, tvec, 0.10)
                Rt,_Jacobian = cv2.Rodrigues(rvec)
                tvec=numpy.transpose(tvec[0])
                tvec=tvec.reshape(3,1)
                temp=numpy.concatenate(Rt)
                temp=temp.reshape(3,3)
                transformation=numpy.append(temp,tvec,axis=1)
                print(transformation)
                # camcord[:,o]=numpy.matmul(transformation,objpoints[4,:])
                o=o+1
        # Display our image
        cv2.imshow('QueryImage', QueryImg)
        cv2.imwrite('det/det_'+str(i)+".png",QueryImg)
        cv2.waitKey(1)


def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/bebop/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback,  queue_size = 20)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    i=0
    main()
