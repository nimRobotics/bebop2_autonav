#! /usr/bin/python

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
import numpy as np
import pandas as pd
import cv2
import cv2.aruco as aruco
import os
import pickle
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from numpy.linalg import inv


def restPovec(id):
   if id == 1:
       mkpos = [0.3048,1.2509,0,1]
   #elif id == 3:
       #mkpos = [0.4255,1.2509,0,1]
   #elif id == 5:
       #mkpos = [0.3048,1.0541,0,1]
   #elif id == 7:
       #mkpos = [0.4255,1.0541,0,1]
   elif id == 9:
       mkpos = [0.8303,1.2287,0,1]
   elif id == 11:
       mkpos = [0.9477,1.2287,0,1]
   elif id == 13:
       mkpos = [0.8303,1.0541,0,1]
   elif id == 15:
       mkpos = [0.9477,1.0541,0,1]
   elif id == 25:
       mkpos = [1.3676,1.2287,0,1]
   elif id == 27:
       mkpos = [1.4859,1.2287,0,1]
   elif id == 29:
       mkpos = [1.3676,1.0541,0,1]
   elif id == 31:
       mkpos = [1.4859,1.0541,0,1]
   else:
       mkpos = [0,0,0,0]
   return mkpos

if not os.path.exists('./calibration4.pckl'):
    print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
    exit()
else:
    f = open('calibration4.pckl', 'rb')
    (cameraMatrix, distCoeffs, _, _) = pickle.load(f)
    f.close()
    if cameraMatrix is None or distCoeffs is None:
        print("Calibration issue. Remove ./calibration.pckl and recalibrate your camera.")
        exit()

# Create constant markers
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_1000) 

# Create vectors we'll be using for rotations and translations for postures
rvecs, tvecs = None, None

# Instantiate CvBridge
bridge = CvBridge()


def pose_estimation(msg):
    
    global num
    global aland
    i = 0
    camx = []
    camy = []
    camz = []

    # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    QueryImg = cv2_img
    ret = True
    if (QueryImg is not None):
                
                # grayscale image
                gray = cv2.cvtColor(QueryImg, cv2.COLOR_BGR2GRAY)
                
                # Detect Aruco markers
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

                # Initialize the camera coordinate
                camcord=np.zeros((3,6))
                
                # Require 8 markers in a photo   i=i+1
                if ids is not None and len(ids) > 0:
                    num = num+1
                    
                    #Estimate the posture from each Aruco marker
                    rvecs, tvecs,_ = aruco.estimatePoseSingleMarkers(corners, 0.07, cameraMatrix, distCoeffs)
  
                    for rvec, tvec in zip(rvecs, tvecs):
                        #QueryImg = aruco.drawAxis(QueryImg, cameraMatrix, distCoeffs, rvec, tvec, 0.1)
                        povec=restPovec(ids[i])
                        Rt,_Jacobian=cv2.Rodrigues(rvec)
                        tvec=np.transpose(tvec[0])
                        tvec=tvec.reshape(3,1)
                        temp=np.concatenate(Rt)
                        temp=temp.reshape(3,3)
                        transformation=np.append(temp,tvec,axis=1)
                        append=np.array([0,0,0,1]).reshape(1,4)
                        transformation=np.append(transformation,append,axis=0)
                        transformation=np.linalg.inv(transformation)
                        # Get camera center location in marker coordinate system
                        arucocord=np.matmul(transformation,np.array([0,0,0,1]))
                        # Get camera center location in real world coordinate system
                        cameracord=arucocord+povec
                        cameracord=cameracord[0:3]
                       
                        i=i+1
                        
                        # For display, x is toward the camera, y is along the closet, z is the height
                        camx=np.append(camx,cameracord[2])
                        camy=np.append(camy,cameracord[0])
                        camz=np.append(camz,cameracord[1])
                    # Write the photo     
                    cv2.imwrite(str(num)+".png",QueryImg)     ## should check every time to avoid overwrite  
                else:
                    print("no image detected")
                    aland = 1
                camcord = [camx,camy,camz,aland]
    return camcord              


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

# The real world xyz system is consistant with the marker coordinate system
# In the marker system, y is the height, x is along the closet, z is toward the camera 

def prediction2d(x,vx):
    A = np.array([[1,1],
                  [0,1]])
    X = np.array([[x],
                  [vx]])
    X_prime = A.dot(X)       # Here we don't consider adding acceleration a
    return X_prime

def covariance2d(sigma_x, sigma_vx):
    sigma = np.array([[sigma_x, sigma_vx]])
    cov_matrix = (sigma.T).dot(sigma)
    return cov_matrix              


def main():
  
    kf_camx = []
    kf_camy = []
    kf_camz = []
    kf_vx = []
    kf_vy = []
    a_x = []
    a_y = []
    a_z = []
    move_time = []
    
    
    global land_pub
    global aland
    global num
    
    camx = []
    camy = []
    camz = []
    num = 0
    speedx = 0
    speedy = 0
    target_x = 1.65 * 100
    target_y = 0.8 * 100
    target_z = 1.35 * 100
    
    # Initial Estimation Covariance Matrix
    # Process / Estimation Errors
    error_est_x = 0.01*100
    error_est_vx = 0.005*100
    error_est_y = 0.05*100
    error_est_vy = 0.025*100
    error_est_z = 0*100
    error_est_vz = 0*100

    # Observation Errors (To avoid S being singular matrix, cannot use the same multiply)
    error_obs_x = 5 *100 # Uncertainty in the measurement
    error_obs_vx = 10 *100
    error_obs_y = 0.1 *100
    error_obs_vy = 0.2 *100
    error_obs_z = 0.2 *100
    error_obs_vz = 0.4 *100

    A = np.array([[1,1],
                  [0,1]])
    H = np.identity(2) 
    Px = covariance2d(error_est_x, error_est_vx)
    Rx = covariance2d(error_obs_x,error_obs_vx)
    Py = covariance2d(error_est_y, error_est_vy)
    Ry = covariance2d(error_obs_y,error_obs_vy)
    Pz = covariance2d(error_est_z, error_est_vz)
    Rz = covariance2d(error_obs_z,error_obs_vz)
    
    # Get initial location
    image_topic = "/bebop/image_raw"
    msg = rospy.wait_for_message(image_topic, Image)
    camcord = pose_estimation(msg)
    camx = camcord[0]
    camy = camcord[1]
    camz = camcord[2]
    aland = camcord[3]
    if(aland == 0):
        image_topic = "/bebop/image_raw"
        msg = rospy.wait_for_message(image_topic, Image)
        camcord = pose_estimation(msg)
        camx = np.append(camx,camcord[0])
        camy = np.append(camy,camcord[1])
        camz = np.append(camz,camcord[2])
        aland = camcord[3]
    if(aland == 1):
       land_pub = rospy.Publisher("bebop/land",Empty,queue_size=1)
       land()

    avg_x = np.mean(camx) * 100
    avg_y = np.mean(camy) * 100
    avg_z = np.mean(camz) * 100
    pos_x = avg_x
    pos_y = avg_y
    pos_z = avg_z
    a_x = np.append(a_x,avg_x/100)
    a_y = np.append(a_y,avg_y/100)
    a_z = np.append(a_z,avg_z/100)
    
    with open('camx.txt','a') as fz:
            np.savetxt(fz, camx, delimiter=",",fmt='%.4f')
    with open('camy.txt','a') as fz:
            np.savetxt(fz, camy, delimiter=",",fmt='%.4f')
    with open('camz.txt','a') as fz:
            np.savetxt(fz, camz, delimiter=",",fmt='%.4f')

    # initial kalman filter state (x,y,z,vx,vy,vz)
    X = np.array([[pos_x],
                  [-3]])        # !!!The direction of speed in kalman filter is the reverse of the drone's speed
    Y = np.array([[pos_y],
                  [0]])
    Z = np.array([[pos_z],
                  [0]])
    kf_camx = np.append(kf_camx, X[0][0]/100)
    kf_camy = np.append(kf_camy, Y[0][0]/100)
    kf_camz = np.append(kf_camz, Z[0][0]/100)
    kf_vx = np.append(kf_vx, X[1][0]/100)
    kf_vy = np.append(kf_vy, Y[1][0]/100)
    
    current_x = X[0,0]      # all elements are x*100 (unit:cm)

    #True or False
    isMove = input("Sure to move x?: ")
    if(isMove):
      
      # Move on x direction (towards the marker)
      while  (current_x < target_x-20 or current_x > target_x-20):                                     ####
         
         # within a safe x range (just pose estimation result, in case kalman filter not good)
         if (pos_x > target_x - 100  and pos_x < target_x + 100 and aland == 0):
           # within a safe y range
           if (pos_y > target_y - 100  and pos_y < target_y + 100 and aland == 0):              ####
               
               prepose_x = pos_x
               prepose_y = pos_y               
               prepose_z = pos_z

               # set speed
               if (current_x < target_x-20):
                   speedx = -0.01                      #!Notice that the coordinate of world and drone are reversed
                   distance = target_x-20-current_x
               if (current_x > target_x-20):
                   speedx = 0.03                       #!Notice that the coordinate of world and drone are reversed
                   distance = current_x-target_x              
 
               # Prediction in Kalman filter
               X = prediction2d(X[0][0], X[1][0])
               Y = prediction2d(Y[0][0], Y[1][0])
               Z = prediction2d(Z[0][0], Z[1][0])
               Px = A.dot(Px).dot(A.T)
               Py = A.dot(Py).dot(A.T)
               Pz = A.dot(Pz).dot(A.T)
               
               # Calculate the Kalman gain
               Sx = H.dot(Px).dot(H.T) + Rx +0.0001*H     # H represents the random error
               Sy = H.dot(Py).dot(H.T) + Ry +0.0001*H
               Sz = H.dot(Pz).dot(H.T) + Rz +0.0001*H                      
               Kx = Px.dot(H.T).dot(inv(Sx))
               Ky = Py.dot(H.T).dot(inv(Sy))
               Kz = Pz.dot(H.T).dot(inv(Sz))
             
               # Move 
               vel_msg.linear.x = speedx
               vel_msg.linear.y = 0
               vel_msg.linear.z = 0
               vel_msg.angular.x = 0
               vel_msg.angular.y = 0
               vel_msg.angular.z = 0
               # Set the timer
               t0 = rospy.Time.now().to_sec()
               t1 = rospy.Time.now().to_sec()
               while(t1-t0 <= 1):       # either move 2 secs 
                     #Publish the velocity
                     velocity_publisher.publish(vel_msg)
                     #Takes actual time to velocity calculus
                     t1=rospy.Time.now().to_sec()
           
               # stop the robot
               vel_msg.linear.x = 0
               velocity_publisher.publish(vel_msg)
               t2=rospy.Time.now().to_sec()
               move_time = np.append(move_time,t2-t0)
               with open('move_time.txt','a') as fx:
                    np.savetxt(fx, move_time, delimiter=",",fmt='%.6f')
            
               # Pose estimation (measurement)
               image_topic = "/bebop/image_raw"
               msg = rospy.wait_for_message(image_topic, Image)
               camcord = pose_estimation(msg)
               camx = camcord[0]
               camy = camcord[1]
               camz = camcord[2]
               aland = camcord[3]
               if(aland == 0):
                    image_topic = "/bebop/image_raw"
                    msg = rospy.wait_for_message(image_topic, Image)
                    camcord = pose_estimation(msg)
                    camx = np.append(camx,camcord[0])
                    camy = np.append(camy,camcord[1])
                    camz = np.append(camz,camcord[2])
                    aland = camcord[3]
               if(aland == 1):
                    land_pub = rospy.Publisher("bebop/land",Empty,queue_size=1)
                    land()  
               avg_x = np.mean(camx) * 100
               avg_y = np.mean(camy) * 100
               avg_z = np.mean(camz) * 100
               pos_x = avg_x
               pos_y = avg_y
               pos_z = avg_z
               a_x = np.append(a_x,avg_x/100)
               a_y = np.append(a_y,avg_y/100)
               a_z = np.append(a_z,avg_z/100)
               
               with open('camx.txt','a') as fz:
                    np.savetxt(fz, camx, delimiter=",",fmt='%.4f')
               with open('camy.txt','a') as fz:
                    np.savetxt(fz, camy, delimiter=",",fmt='%.4f')
               with open('camz.txt','a') as fz:
                    np.savetxt(fz, camz, delimiter=",",fmt='%.4f')

               # Update in Kalman filter 
               datax = [pos_x,pos_x-prepose_x]
               datay = [pos_y,pos_y-prepose_y]
               dataz = [pos_z,pos_z-prepose_z]
               Mx = H.dot(datax).reshape(2, -1)
               My = H.dot(datay).reshape(2, -1)
               Mz = H.dot(dataz).reshape(2, -1)
               X = X + Kx.dot(Mx - H.dot(X))
               Y = Y + Ky.dot(My - H.dot(Y))
               Z = Z + Kz.dot(Mz - H.dot(Z))
               Px = (np.identity(len(Kx)) - Kx.dot(H)).dot(Px)
               Py = (np.identity(len(Ky)) - Ky.dot(H)).dot(Py)
               Pz = (np.identity(len(Kz)) - Kz.dot(H)).dot(Pz)

               current_x = X[0,0]
               kf_camx = np.append(kf_camx, X[0][0]/100)
               kf_camy = np.append(kf_camy, Y[0][0]/100)
               kf_camz = np.append(kf_camz, Z[0][0]/100)
               kf_vx = np.append(kf_vx, X[1][0]/100)
               kf_vy = np.append(kf_vy, Y[1][0]/100)
           else:
             print("y is out of the safe range")
             current_x = target_x
             Y[0,0] = target_y
         else:
           print("x is out of the safe range")
           current_x = target_x
           Y[0,0] = target_y

      print("finish moving x, now move on y")
      # initial kalman filter state (x,y,z,vx,vy,vz)
      X = np.array([[X[0,0]],
                  [0]])        # !!!The direction of speed in kalman filter is the reverse of the drone's speed
      Y = np.array([[Y[0,0]],
                  [-8]])
      Z = np.array([[Z[0,0]],
                  [0]])
      kf_vx = np.append(kf_vx, X[1][0]/100)
      kf_vy = np.append(kf_vy, Y[1][0]/100)
      error_est_x = 0.00005*100
      error_est_vx = 0*100
      error_est_y = 0.001*100
      error_est_vy = 0.0005*100
      Px = covariance2d(error_est_x, error_est_vx)
      Py = covariance2d(error_est_y, error_est_vy)
      current_y = Y[0,0]
      
      # Move on y direction    
      while(current_y < target_y-20 or current_y > target_y+20):
         
         # within a safe x range (just pose estimation result, in case kalman filter not good)
         if (X[0,0] > target_x - 100  and X[0,0] < target_x + 100 and aland == 0):
           # within a safe y range
           if (pos_y > target_y - 20  and pos_y < target_y + 20 and aland == 0):              ####
               
               prepose_x = pos_x
               prepose_y = pos_y               
               prepose_z = pos_z

               # set speed
               if (current_y < target_y-20):
                   speedy = -0.01                      #!!!!Notice that the coordinate of world and drone are reversed
                   distance = target_y-20-current_y
               if (current_y > target_y+20):
                   speedy = 0.02                       #!!!!Notice that the coordinate of world and drone are reversed
                   distance = current_y-target_y-20              
 
               # Prediction in Kalman filter
               X = prediction2d(X[0][0], X[1][0])
               Y = prediction2d(Y[0][0], Y[1][0])
               Z = prediction2d(Z[0][0], Z[1][0])
               Px = A.dot(Px).dot(A.T)
               Py = A.dot(Py).dot(A.T)
               Pz = A.dot(Pz).dot(A.T)
               
               # Calculate the Kalman gain
               Sx = H.dot(Px).dot(H.T) + Rx +0.0001*H     # H represents the random error
               Sy = H.dot(Py).dot(H.T) + Ry +0.0001*H
               Sz = H.dot(Pz).dot(H.T) + Rz +0.0001*H                      
               Kx = Px.dot(H.T).dot(inv(Sx))
               Ky = Py.dot(H.T).dot(inv(Sy))
               Kz = Pz.dot(H.T).dot(inv(Sz))
             
               # Move 
               vel_msg.linear.x = 0
               vel_msg.linear.y = speedy
               vel_msg.linear.z = 0
               vel_msg.angular.x = 0
               vel_msg.angular.y = 0
               vel_msg.angular.z = 0
               # Set the timer
               t0 = rospy.Time.now().to_sec()
               t1 = rospy.Time.now().to_sec()
               while(t1-t0 <= 1):       # either move 2 secs or move to target
                     #Publish the velocity
                     velocity_publisher.publish(vel_msg)
                     #Takes actual time to velocity calculus
                     t1=rospy.Time.now().to_sec()
           
               # stop the robot
               vel_msg.linear.y = 0
               velocity_publisher.publish(vel_msg)
               t2=rospy.Time.now().to_sec()
               move_time = np.append(move_time,t2-t0)
               with open('move_time.txt','a') as fx:
                    np.savetxt(fx, move_time, delimiter=",",fmt='%.6f')
            
               # Pose estimation (measurement)
               image_topic = "/bebop/image_raw"
               msg = rospy.wait_for_message(image_topic, Image)
               camcord = pose_estimation(msg)
               camx = camcord[0]
               camy = camcord[1]
               camz = camcord[2]
               aland = camcord[3]
               if(aland == 0):
                    image_topic = "/bebop/image_raw"
                    msg = rospy.wait_for_message(image_topic, Image)
                    camcord = pose_estimation(msg)
                    camx = np.append(camx,camcord[0])
                    camy = np.append(camy,camcord[1])
                    camz = np.append(camz,camcord[2])
                    aland = camcord[3]
               if(aland == 1):
                    land_pub = rospy.Publisher("bebop/land",Empty,queue_size=1)
                    land()     
    
               avg_x = np.mean(camx) * 100
               avg_y = np.mean(camy) * 100
               avg_z = np.mean(camz) * 100
               pos_x = avg_x
               pos_y = avg_y
               pos_z = avg_z
               a_x = np.append(a_x,avg_x/100)
               a_y = np.append(a_y,avg_y/100)
               a_z = np.append(a_z,avg_z/100)
               
               with open('camx.txt','a') as fz:
                    np.savetxt(fz, camx, delimiter=",",fmt='%.4f')
               with open('camy.txt','a') as fz:
                    np.savetxt(fz, camy, delimiter=",",fmt='%.4f')
               with open('camz.txt','a') as fz:
                    np.savetxt(fz, camz, delimiter=",",fmt='%.4f')

               # Update in Kalman filter 
               datax = [pos_x,pos_x-prepose_x]
               datay = [pos_y,pos_y-prepose_y]
               dataz = [pos_z,pos_z-prepose_z]
               Mx = H.dot(datax).reshape(2, -1)
               My = H.dot(datay).reshape(2, -1)
               Mz = H.dot(dataz).reshape(2, -1)
               X = X + Kx.dot(Mx - H.dot(X))
               Y = Y + Ky.dot(My - H.dot(Y))
               Z = Z + Kz.dot(Mz - H.dot(Z))
               Px = (np.identity(len(Kx)) - Kx.dot(H)).dot(Px)
               Py = (np.identity(len(Ky)) - Ky.dot(H)).dot(Py)
               Pz = (np.identity(len(Kz)) - Kz.dot(H)).dot(Pz)

               current_y = Y[0,0]
               kf_camx = np.append(kf_camx, X[0][0]/100)
               kf_camy = np.append(kf_camy, Y[0][0]/100)
               kf_camz = np.append(kf_camz, Z[0][0]/100)
               kf_vx = np.append(kf_vx, X[1][0]/100)
               kf_vy = np.append(kf_vy, Y[1][0]/100)
           else:
             print("y is out of the safe range")
             current_y = target_y
         else:
           print("x is out of the safe range")
           current_y = target_y


    with open('kf_camx.txt','a') as fz:
          np.savetxt(fz, kf_camx, delimiter=",",fmt='%.4f')
    with open('kf_camy.txt','a') as fz:
          np.savetxt(fz, kf_camy, delimiter=",",fmt='%.4f')
    with open('kf_camz.txt','a') as fz:
          np.savetxt(fz, kf_camz, delimiter=",",fmt='%.4f')
    with open('avgx.txt','a') as fx:
          np.savetxt(fx, a_x, delimiter=",",fmt='%.4f')
    with open('avgy.txt','a') as fy:
          np.savetxt(fy, a_y, delimiter=",",fmt='%.4f')
    with open('avgz.txt','a') as fz:
          np.savetxt(fz, a_z, delimiter=",",fmt='%.4f')
    with open('kf_vx.txt','a') as fz:
          np.savetxt(fz, kf_vx, delimiter=",",fmt='%.4f')
    with open('kf_vy.txt','a') as fz:
          np.savetxt(fz, kf_vy, delimiter=",",fmt='%.4f')
    print("finish moving")
    time.sleep(1)


if __name__ == '__main__':
 
    global aland
    aland = 0 
  
    # Starts a new node
    rospy.init_node('parrot_bebop2', anonymous=True)

    # publisher for takeoff
    takeoff_pub = rospy.Publisher("bebop/takeoff",Empty,queue_size=1)
    isTakeoff = input("Sure to takeoff?: ")   #True or False
    if (isTakeoff):
       takeoff()
       time.sleep(4)    #3s for takeoff
       
       # Fly to the planned height
       velocity_publisher = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
       vel_msg = Twist()
       current_height = 0
       height = 0.36           #m 
       speedz = 0.06           #m/s
       #isMove = input("Sure to move z?: ")    #True or False
       isMove = True
       if (isMove):
          vel_msg.linear.x = 0
          vel_msg.linear.y = 0
          vel_msg.linear.z = speedz
          vel_msg.angular.x = 0
          vel_msg.angular.y = 0
          vel_msg.angular.z = 0
          t0 = rospy.Time.now().to_sec()
          while(current_height < height):       #distance can substract a certain number to allow drift
                 #Publish the velocity
                 velocity_publisher.publish(vel_msg)
                 #Takes actual time to velocity calculus
                 t1=rospy.Time.now().to_sec()
                 #Calculates distancePoseStamped
                 current_height= speedz*(t1-t0)
    
       # Reached the height, stop
       vel_msg.linear.z = 0
       velocity_publisher.publish(vel_msg)
       time.sleep(1)

       # Move and collect photos
       main()
        
       #Land the robot
       #landing = input("Ready to land?: ")   #True or False
       landing = True
       print("start landing!")
       if (landing and aland == 0):
           landspeed = 0.06
           landdistance = 0.54
           currentz = 0
           vel_msg.linear.z = -landspeed
           t2 = rospy.Time.now().to_sec()
           while(currentz < landdistance):
                 velocity_publisher.publish(vel_msg)
                 t3=rospy.Time.now().to_sec()
                 currentz = landspeed*(t3-t2)
           vel_msg.linear.z = 0
           velocity_publisher.publish(vel_msg)
           # publisher for landing
           land_pub = rospy.Publisher("bebop/land",Empty,queue_size=1)
           land()
       else:
           print("already landed!")
       
  












