#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

import random

##create cv2 bridge
bridge = CvBridge()


##publish to gazebo/model_states 
def publish_to_gazebo(x,y, z, yaw):
    rospy.init_node('teleport_robot', anonymous=True)
    rospy.wait_for_service('/gazebo/set_model_state')
    #create handle for the service
    pub  = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    #create a message
    msg = ModelState()
    #set the model name
    msg.model_name = 'R1'
    #set the pose
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = 0
    msg.pose.orientation.y = 0
    msg.pose.orientation.z = yaw
    msg.pose.orientation.w = yaw
    #set the twist
    msg.twist.linear.x = 0
    msg.twist.linear.y = 0
    msg.twist.linear.z = 0
    msg.twist.angular.x = 0
    msg.twist.angular.y = 0
    msg.twist.angular.z = 0
    #publish the message
    pub(msg)
    #wait for 1 second
    rospy.sleep(0.1)

publish_to_gazebo(1.252, 0.853, 0.2, 0.09)
# while not rospy.is_shutdown():
#     x = random.uniform(-1.28, 1.28)
#     y = random.uniform(-1.28, 1.28)
#     z = 0.1
#     yaw = random.uniform(-3.14, 3.14)
#     publish_to_gazebo(x,y,z,yaw)
#     rospy.sleep(20)