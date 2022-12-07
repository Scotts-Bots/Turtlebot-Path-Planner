#!/usr/bin/env python

import sys
import rospy
from gazebo_msgs.srv import GetModelState
from math import atan2,pi
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Empty
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pic import *

gx = 0
gy = 0
gz = 0
currx = 0.0
curry = 0.0
angle = 0.0

#function that moves to specfied point
def MoveTo(x, y):
    
    rospy.init_node("pathfinder" , anonymous = True)
    empty_msg = Empty()
    rate = rospy.Rate(120)
    mover = rospy.Publisher('/mobile_base/commands/velocity', Twist , queue_size=1) 
    stop = rospy.Publisher('/mobile_base/commands/velocity', Twist , queue_size=1)
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    twist2 = Twist()
    twist2.linear.x = 0.0
    twist2.linear.y = 0.0
    twist2.linear.z = 0.0
    twist2.angular.x = 0.0
    twist2.angular.y = 0.0
    twist2.angular.z = 0.0
    
    found = False

    droneState = rospy.ServiceProxy('gazebo/get_model_state', GetModelState())
    data = droneState('mobile_base','')
    currx = data.pose.position.x
    curry = data.pose.position.y
    oList = [data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z, data.pose.orientation.w ]
    (roll,pitch,yaw) = euler_from_quaternion(oList)
    target = atan2(y-curry , x - currx)

    while (abs(target-yaw) > 0.01):
        twist.angular.z = (target-yaw)*0.4
        twist.linear.x = 0.0
        mover.publish(twist)
        droneState = rospy.ServiceProxy('gazebo/get_model_state', GetModelState())
        data = droneState('mobile_base','')
        currx = data.pose.position.x
        curry = data.pose.position.y
        oList = [data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z, data.pose.orientation.w ]
        (roll,pitch,yaw) = euler_from_quaternion(oList)
        target = atan2(y-curry , x - currx)

    stop.publish(twist2)

    while found == False:
        droneState = rospy.ServiceProxy('gazebo/get_model_state', GetModelState())
        data = droneState('mobile_base','')
        currx = data.pose.position.x
        curry = data.pose.position.y
        if (abs(x - currx) < 0.1 and abs(y - curry) < 0.1):
            stop.publish(twist2)
            return
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.33
            mover.publish(twist)

#Loops through list of moves and calls MoveTo on each move
#when it reaches the goal it calls takePic to take a picture then checks if 
#the green utility cart is in the surrondings of the cart and publishes an answer to the witsdetector topic
def MoveList(mlist):
    check = False
    # print(type(mlist))
    for i in mlist:
        MoveTo((i[0]-132.0)/10.0,(i[1]-30.0)/10.0)
        # time.sleep(2)
    print('Goal Reached')
    print('Checking 4 sides to detect cart...')
    takePic()
    i = 0
    while i < 4 and not check:
        fname = "pic" + str(i+1) + ".png"
        check = check or detectUtilCart(fname)
        i = i + 1
    rate = rospy.Rate(5)
    pub = rospy.Publisher('witsdetector', String, queue_size=1)
    i = 0
    while i<2:
        if check:
            pub.publish("Yes")
        else:
            pub.publish("No")
        rate.sleep()
        i = i+1
