#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import time

def move1():
    # 5s wait for takeoff to finish
    time.sleep(5)
    vel_msg = Twist()
    print("Let's move your robot")
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0.4
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    while velocity_pub.get_num_connections()<1:
        rospy.loginfo_throttle(2,"waiting for movement")
        rospy.sleep(0.1)

    init_time = time.time()
    dt=0
    while dt<0.7:
        velocity_pub.publish(vel_msg)
        dt = time.time()-init_time

    #
    # init_time = time.time()
    # dt=0
    # while dt<1.2:
    #     velocity_pub.publish(vel_msg)
    #     dt = time.time()-init_time
    #
    # vel_msg.linear.x = 0
    # vel_msg.angular.z = -0.5 #TURRNING AT 0 1
    #
    # init_time = time.time()
    # dt=0
    # while dt<1.8:
    #     velocity_pub.publish(vel_msg)
    #     dt = time.time()-init_time
    #
    # vel_msg.linear.x = 0.2
    # vel_msg.angular.z = 0
    #
    # init_time = time.time()
    # dt=0
    # while dt<1.5:
    #     velocity_pub.publish(vel_msg)
    #     dt = time.time()-init_time
    #
    # vel_msg.linear.x = 0
    # vel_msg.angular.z = -0.5 #TURRNING AT 1 1
    #
    # init_time = time.time()
    # dt=0
    # while dt<1.8:
    #     velocity_pub.publish(vel_msg)
    #     dt = time.time()-init_time
    #
    # vel_msg.linear.x = 0.2
    # vel_msg.angular.z = 0
    #
    # init_time = time.time()
    # dt=0
    # while dt<1.7:
    #     velocity_pub.publish(vel_msg)
    #     dt = time.time()-init_time
    #
    # vel_msg.linear.x = 0
    # vel_msg.angular.z = -0.5 #TURRNING AT 1 0
    #
    # init_time = time.time()
    # dt=0
    # while dt<1.8:
    #     velocity_pub.publish(vel_msg)
    #     dt = time.time()-init_time
    #
    # vel_msg.linear.x = 0.2
    # vel_msg.angular.z = 0
    #
    # init_time = time.time()
    # dt=0
    # while dt<1.7:
    #     velocity_pub.publish(vel_msg)
    #     dt = time.time()-init_time
    #
    # vel_msg.linear.x = 0
    # vel_msg.angular.z = -0.5 #TURRNING AT 0 0
    #
    # init_time = time.time()
    # dt=0
    # while dt<1.8:
    #     velocity_pub.publish(vel_msg)
    #     dt = time.time()-init_time



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

if __name__ == '__main__':
    try:
        # intializing the node
        rospy.init_node('parrot_control',anonymous=True)

        # publisher for takeoff
        takeoff_pub = rospy.Publisher("bebop/takeoff",Empty,queue_size=1)
        takeoff()
        # publisher for velocity
        velocity_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        move1()
        # publisher for landing
        land_pub = rospy.Publisher("bebop/land",Empty,queue_size=1)
        land()
    except rospy.ROSInterruptException: pass
