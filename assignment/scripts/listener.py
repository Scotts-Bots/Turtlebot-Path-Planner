#!/usr/bin/env python

import rospy
from std_msgs.msg import String

#witsdetector topic listens for publisher from pos.py to output if breen utility cart was found
def callback(data):
    rospy.loginfo("Cart detected: " + data.data)

def listen():

    rospy.init_node('witsdetectorNode', anonymous=True)

    rospy.Subscriber('witsdetector', String, callback)

    rospy.spin()

if __name__ == '__main__':
    listen()
