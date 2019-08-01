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
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
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


def land():
    rate = rospy.Rate(10)
    while land_pub.get_num_connections()<1:
        rospy.loginfo_throttle(2,"waiting for landing")
        rospy.sleep(0.1)
    land_pub.publish(Empty())

def takeoff():
    rate = rospy.Rate(10)
    while takeoff_pub.get_num_connections()<1:
        rospy.loginfo_throttle(2,"waiting for takeoff")
        rospy.sleep(0.1)
    takeoff_pub.publish(Empty())

def move1(lx,ly,lz,ax,ay,az):
    # 5s wait for takeoff to finish
    # time.sleep(5)
    vel_msg = Twist()
    print("Let's move your robot")
    vel_msg.linear.x = lx
    vel_msg.linear.y = ly
    vel_msg.linear.z = lz
    vel_msg.angular.x = ax
    vel_msg.angular.y = ay
    vel_msg.angular.z = az

    while velocity_pub.get_num_connections()<1:
        rospy.loginfo_throttle(2,"waiting for movement")
        rospy.sleep(0.1)
    velocity_pub.publish(vel_msg)

def image_callback(msg):
    start_time = time.time()
    print("Received an image frame 1 !")
    global i
    global j
    global pos_x
    global pos_y
    global pos_z

    if i%6==0: #skippping 6 frames
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            QueryImg = cv2_img
            ret = True
            if ret == True:
                # grayscale image
                gray = cv2.cvtColor(QueryImg, cv2.COLOR_BGR2GRAY)
                # Detect Aruco markers
                # start_time = time.time()
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
                            # print(posavg)
                            pos_x=posavg[0]
                            pos_y=posavg[1]
                            pos_z=posavg[2]
                            # Set up your subscriber and define its callback
                            targetx=0.762
                            targety=1.5742
                            targetz=1.6742
                            print("pos_x",pos_x,"pos_y",pos_y,"pos_z",pos_z)
                            # if pos_x<targetx-0.2 or pos_x>targetx+0.2:
                            global g
                            if pos_z<targetz-0.1 or pos_z>targetz+0.1:
                                print("move zzzz")
                                # dy=
                                move1(0.03,-0.008 ,0,0,0,0)
                            else:
                                if g==0:
                                    print("delay 5 once")
                                    time.sleep(5)
                                    g=1
                                elif pos_x<targetx-0.2 or pos_x>targetx+0.2:
                                    print("move XXX")
                                    move1(0,0.01,0,0,0,0)
                                else:
                                    land()
                                    time.sleep(30)
                            arucopos[i,:]=posavg
                        else:
                            posavg = cameracord+posavg
                    i=0
        print("computation time",time.time()-start_time)
        print(i)
    i=i+1

def main():
    # rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/bebop/image_raw"
    global subsFr1
    subsFr1=rospy.Subscriber(image_topic, Image, image_callback,  queue_size = 1)
    # print("pos_x",pos_x,"pos_y",pos_y,"pos_z",pos_z)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    i=0
    j=0
    g=0
    pos_x=0
    pos_y=0
    pos_z=0

    # intializing the node
    rospy.init_node('parrot_control',anonymous=True)
    # publisher for takeoff
    takeoff_pub = rospy.Publisher("bebop/takeoff",Empty,queue_size=1)
    takeoff()

    # publisher for velocity
    velocity_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)

    # 5s wait for takeoff to finish
    time.sleep(5)
    vel_msg = Twist()
    print("Let's move your robot")
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0.12
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    while velocity_pub.get_num_connections()<1:
        rospy.loginfo_throttle(2,"waiting for movement")
        rospy.sleep(0.1)

    init_time = time.time()
    dt=0
    while dt<5.5:
        velocity_pub.publish(vel_msg)
        dt = time.time()-init_time
    time.sleep(5)


    # move1()
    # publisher for landing
    land_pub = rospy.Publisher("bebop/land",Empty,queue_size=1)
    # land()

    main()
