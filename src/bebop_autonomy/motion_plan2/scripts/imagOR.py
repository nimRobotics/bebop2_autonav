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
        # Display our image
        # cv2.imshow('QueryImage', cv2_img)
        cv2.imwrite('photos/'+str(i)+".png",cv2_img)
        # cv2.waitKey(1)


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
