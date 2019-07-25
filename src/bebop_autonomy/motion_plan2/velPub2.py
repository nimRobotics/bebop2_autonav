#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import time

def move1():
    time.sleep(5)
    vel_msg = Twist()
    print("Let's move your robot")
    vel_msg.linear.x = 0
    vel_msg.linear.y = -1
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    while velocity_publisher.get_num_connections()<1:
        rospy.loginfo_throttle(2,"waiting for movement")
        rospy.sleep(0.1)
    velocity_publisher.publish(vel_msg)
    # time.sleep(1.5)
    # vel_msg.angular.z = -1
    velocity_publisher.publish(vel_msg)

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
        #Testing our functionvelocity_publisher.publish(vel_msg)
        # move()
        takeoff_pub = rospy.Publisher("bebop/takeoff",Empty,queue_size=1)
        rospy.init_node('takfdvesdoff',anonymous=True)
        takeoff()

        velocity_publisher = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        move1()

        land_pub = rospy.Publisher("bebop/land",Empty,queue_size=1)
        land()
    except rospy.ROSInterruptException: pass
