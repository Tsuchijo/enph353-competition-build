#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import GetModelState
import os

import random

##create cv2 bridge
bridge = CvBridge()
image_name  = 'image.png'
path = '/home/fizzer/353_comp_training/training_data2'

pts_source = np.array([[180, 218], [455, 0], [1141, 219], [835, 0]])
pts_destination = np.array([[180, 218], [180, 0], [1141, 219], [1141, 0]])
h, status = cv2.findHomography(pts_source, pts_destination)

##subscribe to the camera
def callback(data):
    #convert the image to cv2 format
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    image_cutoff = 500
    im_dst = cv2.warpPerspective(cv_image[image_cutoff:, ::], h, (cv_image[image_cutoff:, ::].shape[1], cv_image[image_cutoff:, ::].shape[0]))

    # get the model state
    state = get_model_state()
    # get the z twist and foward velocity
    z_twist = state.twist.angular.z
    foward_vel = np.sqrt(state.twist.linear.x**2 + state.twist.linear.y**2)
    timestamp = rospy.get_time()
    #truncate the values to 3 decimal places
    z_twist = round(z_twist, 3)
    foward_vel = round(foward_vel, 3)
    filename = os.path.join(path, str(z_twist) + ',' + str(foward_vel) + ',' + str(timestamp)+ '.png')
    #downasample the image
    cv_image = cv2.resize(im_dst, (320, 180))
    #save the image
    cv2.imwrite(filename, cv_image)
    print('x y pose: ', state.pose.position.x, state.pose.position.y, state.pose.position.z)
    #rospy.sleep(0.1)

## get the model state 
def get_model_state():
    #wait for the service
    rospy.wait_for_service('/gazebo/get_model_state')
    #create handle for the service
    get_state  = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    #create a message
    msg = ModelState()
    #set the model name~
    msg.model_name = 'R1'
    #get the state
    state = get_state('R1', 'world')
    #return the state
    return state

##subscribe to the camera
def subscribe_to_camera():
    rospy.init_node('get_training_data', anonymous=True)
    rospy.Subscriber("/R1/pi_camera/image_raw", Image, callback)

subscribe_to_camera()

## loop to get the data
while not rospy.is_shutdown():
    rospy.spin()