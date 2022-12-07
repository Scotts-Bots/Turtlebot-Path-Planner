#!/usr/bin/env python

import sys
import rospy
from gazebo_msgs.srv import GetModelState
from math import atan2
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import numpy as np
from pos import *
from FindPath import *

#main function takes in coords and calls path planning function that returns list of moves
#that gets passed to the moveList function
if __name__ == "__main__":
    
    droneState = rospy.ServiceProxy('gazebo/get_model_state', GetModelState())
    print("Enter co-ordinates for robot to move to x = {0,1499}, y = {0,1999} and use -1 to stop")
    print("Enter the x value first:")
    data = droneState('mobile_base','')
    start = ((int(data.pose.position.x)*10)+132, (int(data.pose.position.y)*10)+30)
    gx = input()
    if (gx == -1):
        exit()
    print("Enter y:")
    gy = input()
    if (gy == -1):
        exit()
    res = 3000
    rad = 15
    while True:
        print("start: ",start)
        print("Attempting to find a path...")
        moveList = findPath(start,(gx,gy) , res, rad)
        print("Finished attempt")
        if type(moveList) != bool :
            if (moveList == None or len(moveList) == 0):
                print("Trying again with higher resolution...")
                res = res + 2000
                moveList = findPath(start,(gx,gy) , res, rad)
            if (moveList != None):
                print("Moving along path...")
                MoveList(moveList)
                start = (gx,gy)
            else:
                print("Path not found again.")
        print("Enter co-ordinates for robot to move to x = {0,199}, y = {0,149} and use -1 to stop")
        print("Enter the x value first:")
        gx = input()
        if (gx == -1):
            exit()
        print("Enter y:")
        gy = input()
        if (gy == -1):
            exit()