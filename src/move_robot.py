#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
import keras
import os

#create cv2 bridge 
bridge = CvBridge()

##init the keras model
def init_model():
    model = keras.models.load_model("/home/fizzer/ros_ws/src/Enph353-Compition-Controller/src/model2/model2")
    model.summary()
    return model
model = init_model()
# do an inverse perspective transform on the masked image
# take the points from observations of lines
pts_source = np.array([[180, 218], [455, 0], [1141, 219], [835, 0]])
pts_destination = np.array([[180, 218], [180, 0], [1141, 219], [1141, 0]])
h, status = cv2.findHomography(pts_source, pts_destination)
#create state machine
state = 0
vert_angle = 0
horizontal_line = False
avg_line_x = 0
lines_pointing_away = False
lines_bimodal = False
see_car = False
test_mode = False
see_crosswalk = False
inside_crosswalk = False
avg_x = 0
crossing_counter = 0
last_frame = np.zeros((720, 1280, 3), np.uint8)
movement = 0
crosswalk_clear = False
nn_timer = 0
gray_sum = 0


# create a state machine to control the car movement
# state 0: start timer
# state 1: line follow off vertical lines until it sees a horizontal line or the lines point away from eachother
# state 2: decision making state for if it sees a horizontal line, if the line is in the center then it is a left turn or pedestrian
# if the line is to the right or left it is a car, if the lines are to the left and right but not center it is an intersection
# state 4: if it sees a left turn then it will turn left until it no longer sees a horizontal line then go back to state 1
# state 6: if it sees the lines pointing away from eachother then it will turn left until it sees a horizontal line then go back to state 1
# state 7: if it sees a pedestrian then it will stop and wait for the pedestrian to cross then go back to state 
# state 8: if it is inside a crosswalk then it will move forward until it no longer sees a crosswalk then go back to state 1
# state 9: once it has crossed 2 crosswalks it will activate the neural net
def state_machine():
    global state
    global vert_angle
    global horizontal_line
    global avg_line_x
    global lines_pointing_away
    global lines_bimodal
    global see_car
    global see_crosswalk
    global avg_x
    global inside_crosswalk
    global crossing_counter
    global movement
    global crosswalk_clear
    global nn_timer
    speed = 0.08
    nudge_speed = 0.001
    #start at state 0 and start timer then move onto state 
    if state == 0:
        start_timer()
        move_robot(0, 0)
        rospy.sleep(3)
        state = 6
    #move forward until it sees a horizontal line
    if state == 1:
        if horizontal_line == False:
            if lines_pointing_away == False:
                move_robot(speed, (vert_angle * 0.65) - (nudge_speed * (avg_x > 630)) + (nudge_speed * (avg_x < 630)))
                crosswalk_clear = False
            else:
                state = 6
        else:
            state = 2
    #decision making state
    if state == 2:
        if see_crosswalk == True:
            state = 7
        if avg_line_x > 300 and avg_line_x < 1000:
            state = 4
        else: 
            state = 1
    # if it sees a left turn
    if state == 4:
        move_robot(0.01, 1)
        if horizontal_line == False:
            state = 1
        if see_crosswalk == True:
            state = 7
    # if it sees the lines pointing away from eachother
    if state == 6:
        move_robot(speed,  - (nudge_speed * (avg_x > 640)) + (nudge_speed * (avg_x < 640)))
        rospy.sleep(0.5)
        if horizontal_line == True:
            state = 2
    if state == 7:
        if see_crosswalk == True:
            if crosswalk_clear == True:
                move_robot(speed * 4, 0)
            else:
                move_robot(0, 0)
                rospy.sleep(0.5)
                if movement < 70:
                    crosswalk_clear = True
        elif inside_crosswalk == True:
            inside_crosswalk = False
            state = 1
        else:
            inside_crosswalk = True
            state = 8
    if state == 8:
        if see_crosswalk == False:
            move_robot(speed * 8, (vert_angle * 0.7))
        else:
            state = 7
            crossing_counter += 1
            if crossing_counter >= 2:
                state = 9
    if state == 9:
        rospy.sleep(0.1)
        nn_timer += 1
        # if nn_timer >= 100 and gray_sum > 250 and see_car:
        #     state = 10
    if state == 10:
        stop_timer()
        os.system("rosnode kill /topic_publisher")
        os.system("rosnode kill /analyze_image")



##Image callback
def image_callback(msg):
    global vert_angle
    global left_vert_angle
    global right_vert_angle
    global horizontal_line
    global avg_line_x
    global lines_pointing_away
    global lines_bimodal
    global see_car
    global see_crosswalk
    global avg_x
    global last_frame
    global movement
    global state
    global gray_sum

    image_cutoff = 500
    horizontal_threshold = 0.3
    # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")


    im_dst = cv2.warpPerspective(cv2_img[image_cutoff:, ::], h, (cv2_img[image_cutoff:, ::].shape[1], cv2_img[image_cutoff:, ::].shape[0]))
    
    #do a threshold based on the grayscale image
    ret, mask = cv2.threshold(im_dst, 220, 255, cv2.THRESH_BINARY)
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    #crop the image to only the bottom half
    new_cv2_img = cv2_img[image_cutoff - 25 :, ::]
    #create downsampled image
    small_img = cv2.resize(new_cv2_img, (640,245))
    # for each pixel in the image if the blue is 100 more than red and green then it is a car
    car_mask = threshold(small_img)
    #sum all the pixels in the mask
    car_mask_sum = np.sum(car_mask)
    #if the sum is greater than 10 then it is a car
    if (car_mask_sum / 255) > 50:
        see_car = True
    else:
        see_car = False

    #threshold out bright red to detect crosswalks
    red_mask = threshold_crosswalks(small_img)
    #sum all the pixels in the mask
    red_mask_sum = np.sum(red_mask)
    #if the sum is greater than 10 then it is a crosswalk
    if (red_mask_sum) > 10:
        see_crosswalk = True
    else:
        see_crosswalk = False

    #use canny edge detection on the masked image
    edge_mask = cv2.Canny(mask, 100, 200)

    # detect horizontal lines
    lines = cv2.HoughLinesP(edge_mask, 1, np.pi/180, 50, minLineLength=100, maxLineGap=10)
    vertical_lines = lines

    # filter only lines with slope between -0.1 and 0.1
    if lines is not None:
        lines = [line for line in lines if abs((line[0][3] - line[0][1]) / (line[0][2] - line[0][0])) > -horizontal_threshold]
        lines = [line for line in lines if abs((line[0][3] - line[0][1]) / (line[0][2] - line[0][0])) < horizontal_threshold]
    
    # average all the horizontal lines
    if lines is not None:
        if len(lines) > 0:
            avg_line_x = 0
            for line in lines:
                avg_line_x += line[0][0]
                avg_line_x += line[0][2]
                
            avg_line_x = avg_line_x / (len(lines) * 2)

    # check if there enough horizontal lines to be considered a horizontal line
    if lines is not None:
        if len(lines) > 1:
            horizontal_line = True
        else:
            horizontal_line = False

    # filter only lines from vertical lines with slope greater than 5 or less than -5 
    if vertical_lines is not None:
        vertical_lines = [line for line in vertical_lines if abs((line[0][3] - line[0][1]) / (line[0][2] - line[0][0])) > horizontal_threshold]
    
    # average all the vertical lines
    if vertical_lines is not None:
        # take each line and subtract x1 from x2 and y1 from y2
        vertical_lines_x2 = [(line[0][2] - line[0][0]) for line in vertical_lines]
        vertical_lines_y2 = [(line[0][3] - line[0][1]) for line in vertical_lines]
        # take the average of the x2 and y2
        average_x2 = np.mean(vertical_lines_x2)
        average_y2 = np.mean(vertical_lines_y2)
        # construct a line with the average x2 and y2
        average_line = np.array([[0, 0, average_x2, average_y2]])
    else:
        average_line = None

    left_line_out = []
    right_line_out = []
    # check if the vertical lines are pointing away from the center of the image
    if vertical_lines is not None:
        # split the lines into ones on left and right half of image
        left_vertical_lines = [line for line in vertical_lines if line[0][0] < 640]
        right_vertical_lines = [line for line in vertical_lines if line[0][0] > 640]

        # check if there are enough lines on each side
        if len(left_vertical_lines) > 1 and len(right_vertical_lines) > 1:
            # create a list of the left lines pointing out and in
            left_line_out = [line for line in left_vertical_lines if line[0][1] < line[0][3]]
            # create a list of the right lines pointing out and in
            right_line_out = [line for line in right_vertical_lines if line[0][1] > line[0][3]]
            if len(left_line_out) > 0 and len(right_line_out) > 0:
                lines_pointing_away = True
            else:
                lines_pointing_away = False
        else:
            lines_pointing_away = False

    # # draw lines on image
    # if lines is not None:
    #     for line in lines:
    #         x1, y1, x2, y2 = line[0]
    #         cv2.line(im_dst, (x1, y1), (x2, y2), (255, 0, 0), 2)

    # # draw vertical lines on image
    # if vertical_lines is not None:
    #     for line in vertical_lines:
    #         x1, y1, x2, y2 = line[0]
    #         cv2.line(im_dst, (x1, y1), (x2, y2) , (0, 0, 255), 2)
    
    # draw average line on image
    if average_line is not None:
        #check if average is nan
        if np.isnan(average_line).any():
            average_line = np.array([[0, 0, 0, 0]])
            vert_angle = 0
            left_vert_angle = 0
            right_vert_angle = 0
        x1, y1, x2, y2 = average_line[0]
        #cast to int
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        #calculate angle
        if y2 != 0:
            vert_angle = (np.arctan(x2/y2) + vert_angle)/2
        else:
            vert_angle = 0
    else:
        vert_angle = 0

    # find the average position of gray pixels in im_dst
    # threshold the image to find pixels within a certain range
    gray = threshold_hsv(small_img, 0, 255, 0, 10, int(30 * 2.55), int(35 * 2.55))
    # find the average position of the pixels

    avg_x = np.mean(gray[140, ::, 0] * np.arange(0, 640)) / np.mean(gray) * 2
   
    if state == 9:
        gray_sum = np.sum(gray) / 255

    # draw circle at average position
    if not np.isnan(avg_x):
        cv2.circle(im_dst, (int(avg_x), 100), 5, (0, 255, 0), -1)
    else :
        avg_x = 640
    # Display image in a window called "win3"d
    # cv2.imshow('win3', im_dst)
    # cv2.imshow('win4', small_img)
   
    cv2.waitKey(1)
    if state == 9:
        # downasample the image
        cv_image = cv2.resize(cv2_img, (160, 90))
        pred = (model.predict(np.array([cv_image])))
        action = (np.argmax(pred))
        if action == 0:
            move_robot(0.03, 0.3)
        elif action == 1:
            move_robot(0.03, -0.3)
        elif action == 2:
            move_robot(0.1, 0)
        elif action == 3:
            move_robot(0.0, 0)
    if state == 7:
        difference = cv2.subtract(cv2_img, last_frame)
        ret, difference = cv2.threshold(difference, 50, 1, cv2.THRESH_BINARY)
        #convert to grayscale
        difference = cv2.cvtColor(difference, cv2.COLOR_BGR2GRAY)
        movement = np.sum(difference)
        last_frame = cv2_img

##threshold function for finding cars
def threshold(frame):
    low_H = 120
    low_S = 124
    low_V = 0
    high_H = 125
    high_S = 255
    high_V = 255
    k = 31
    #convert the image to HSV and threshold it
    blurred = cv2.GaussianBlur(frame, (k, k), 0)
    frame_HSV = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
    #convert to a 3 channel image so we can stack it with the original image
    frame_threshold = cv2.cvtColor(frame_threshold, cv2.COLOR_GRAY2BGR)
    return frame_threshold

##threshold for finding crosswalks
def threshold_crosswalks(frame):
    low_H = 0
    low_S = 225
    low_V = 180
    high_H = 10
    high_S = 255
    high_V = 255
    k = 31
    #convert the image to HSV and threshold it
    blurred = cv2.GaussianBlur(frame, (k, k), 0)
    frame_HSV = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
    #convert to a 3 channel image so we can stack it with the original image
    frame_threshold = cv2.cvtColor(frame_threshold, cv2.COLOR_GRAY2BGR)
    return frame_threshold

##threshold hsv
def threshold_hsv(frame, h_low, h_high, s_low, s_high, v_low, v_high):
     #convert the image to HSV and threshold it
    blurred = cv2.GaussianBlur(frame, (31, 31), 0)
    frame_HSV = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (h_low, s_low, v_low), (h_high, s_high, v_high))
    #convert to a 3 channel image so we can stack it with the original image
    frame_threshold = cv2.cvtColor(frame_threshold, cv2.COLOR_GRAY2BGR)
    return frame_threshold


## Move Robot node
# use ros to publish to the topic /cmd_vel:
def move_robot(x, z):
    # handle casse where x or z is None or not a number
    if x is None or not isinstance(x, (int, float)):
        x = 0
    if z is None or not isinstance(z, (int, float)):
        z = 0
    twist = Twist()
    twist.linear.x = x
    twist.angular.z = z
    if not test_mode:
        pub.publish(twist)
    rate.sleep()

# start the timer
def start_timer():
    pub_plate.publish(str('Axolotl,1234,0,0'))

# stop the timer 
def stop_timer():
    pub_plate.publish(str('Axolotl,1234,-1,0'))

rospy.init_node('topic_publisher', anonymous=True)
pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, image_callback)
pub_plate = rospy.Publisher('/license_plate', String, queue_size=1)
rate = rospy.Rate(10) # 10hz
#wait 1 second
rospy.sleep(10)

while not rospy.is_shutdown():
    #print(gray_sum)
    state_machine()
