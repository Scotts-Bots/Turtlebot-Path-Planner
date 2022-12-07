#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import GetModelState
from math import atan2,pi
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import cv2

class TakePhoto:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        rospy.sleep(1)

    def callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False

def pic(pname):
    camera = TakePhoto()

    # Take a photo
    img_title = rospy.get_param('~image_title', pname)

    if camera.take_picture(img_title):
        rospy.loginfo("Saved image " + img_title)
    else:
        rospy.loginfo("No images received")

    rospy.sleep(1)

def takePic():

    target = 0

    empty_msg = Empty()
    rate = rospy.Rate(120)
    mover = rospy.Publisher('/mobile_base/commands/velocity', Twist , queue_size=1) 
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    
    droneState = rospy.ServiceProxy('gazebo/get_model_state', GetModelState())
    data = droneState('mobile_base','') 
    oList = [data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z, data.pose.orientation.w ]
    (roll,pitch,yaw) = euler_from_quaternion(oList)
    target1 = 0
    while (abs(target1-yaw) > 0.01):
        twist.angular.z = (target1-yaw)*0.4
        twist.linear.x = 0.0
        mover.publish(twist)
        data = droneState('mobile_base','')
        currx = data.pose.position.x
        curry = data.pose.position.y
        oList = [data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z, data.pose.orientation.w ]
        (roll,pitch,yaw) = euler_from_quaternion(oList)
    pic("pic1.png")

    droneState = rospy.ServiceProxy('gazebo/get_model_state', GetModelState())
    data = droneState('mobile_base','') 
    oList = [data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z, data.pose.orientation.w ]
    (roll,pitch,yaw) = euler_from_quaternion(oList)
    target1 = pi/2
    while (abs(target1-yaw) > 0.01):
        twist.angular.z = (target1-yaw)*0.4
        twist.linear.x = 0.0
        mover.publish(twist)
        data = droneState('mobile_base','')
        currx = data.pose.position.x
        curry = data.pose.position.y
        oList = [data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z, data.pose.orientation.w ]
        (roll,pitch,yaw) = euler_from_quaternion(oList)
    pic("pic2.png")
    droneState = rospy.ServiceProxy('gazebo/get_model_state', GetModelState())
    data = droneState('mobile_base','') 
    oList = [data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z, data.pose.orientation.w ]
    (roll,pitch,yaw) = euler_from_quaternion(oList)
    target1 = pi
    while (abs(target1-yaw) > 0.01):
        twist.angular.z = (target1-yaw)*0.4
        twist.linear.x = 0.0
        mover.publish(twist)
        data = droneState('mobile_base','')
        currx = data.pose.position.x
        curry = data.pose.position.y
        oList = [data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z, data.pose.orientation.w ]
        (roll,pitch,yaw) = euler_from_quaternion(oList)
    pic("pic3.png")
    droneState = rospy.ServiceProxy('gazebo/get_model_state', GetModelState())
    data = droneState('mobile_base','') 
    oList = [data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z, data.pose.orientation.w ]
    (roll,pitch,yaw) = euler_from_quaternion(oList)
    target1 = -pi/2
    while (abs(target1-yaw) > 0.01):
        twist.angular.z = (target1-yaw)*0.4
        twist.linear.x = 0.0
        mover.publish(twist)
        data = droneState('mobile_base','')
        currx = data.pose.position.x
        curry = data.pose.position.y
        oList = [data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z, data.pose.orientation.w ]
        (roll,pitch,yaw) = euler_from_quaternion(oList)
    pic("pic4.png")


def detectUtilCart(img_file):
    img = cv2.imread(img_file)

    #count number of green pixels
    num = 0
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            r = img[i][j][0] < 20
            g = img[i][j][1] > 150
            b = img[i][j][2] < 20
            if r and g and b:
                num += 1

    if num > 20:
        return True
    elif num > 1:
        print("possibly true")
        return True
    else:
        return False
