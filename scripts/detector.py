#!/usr/bin/env python3

# Python packages
import cv2
import numpy as np

# ROS packages (python)
import rospy
from cv_bridge import CvBridge

# Messages
from sensor_msgs.msg import Image

import features

class LineDetector:
    def __init__(self):
        self.video_data = []
        self.video_writer = cv2.VideoWriter('../recordings/detection.avi', cv2.VideoWriter_fourcc(*'DIVX'), 30, (320, 240))
        self.bridge = CvBridge()

        self.detection = Image()
        self.publisher = rospy.Publisher('line_follower', Image, queue_size=1)

    def read_image(self, message):
        '''
        Reads ROS message and turns it into numpy array.
        It also applies a Gaussian blur and turns the blurred image into HSV color format.

        Inputs
        ---
            message: sensor_msgs.msg.Image
                Sensor image message
        '''

        self.image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        # self.blurred = cv2.GaussianBlur(self.image, ksize=(3, 3), sigmaX=.1, sigmaY=.1)
        # self.blurred_hsv = cv2.cvtColor(self.blurred, cv2.COLOR_BGR2HSV)
        self.height, self.width, _ = self.image.shape

    def get_direction(self, message=None, line_color='red', tol=10):
        '''
        Given an image message, it returns the direction the robot must take in order to follow the line.
        It calculates the direction mainly based on the color of the line.

        Inputs
        ---
            message: sensor_msgs.msg.Image
                Sensor image message

            line_color: str
                Line color: "red" or "black"
            
            tol: int, default = 10
                Admitted tolerance to calculate direction

        Outputs
        ---
            dir: int
                Direction the robot must take:
                    Stop = 0; Straight = 1; Left = 2; Right = 3;
        '''
        
        if message:
            self.read_image(message)

        # if line_color == 'red':
        #     lower = np.array([0, 100, 100])
        #     upper = np.array([10, 255, 255])

        # if line_color == 'black':
        #     lower = np.array([0, 0, 0])
        #     upper = np.array([179, 20, 155])

        img = cv2.imread('logo_train.png') # in same directory
        self.train_features = features.getFeatures(img)

        # mask = features.detectFeatures(self.blurred_hsv, self.train_features)
        # mask = cv2.inRange(self.blurred_hsv, lower, upper)
        region = features.detectFeatures(self.image, self.train_features)
        rospy.loginfo(region)

        # search_y = int(self.height*2/5)
        # mask[:search_y,: ] = 0
        # moments = cv2.moments(mask)
        if region is not None:
            # If features are detected, move to the detected position
            box = cv2.boxPoints(region)
            box = np.int0(box)
            cv2.drawContours(self.image, [box], 0, (0, 255, 0), 2)
            # cv2.putText(self.image, 'Detected logo', (rect_x - 2, rect_y - 8), cv2.FONT_HERSHEY_DUPLEX, .4, (0, 255, 0))
            cx = int((box[0][0] + box[2][0]) / 2)  # X-coordinate of feature center
            frame_width = self.image.shape[1]

            self.detection = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            self.publisher.publish(self.detection)

            self.video_data.append(self.image)

        else:
            cv2.putText(self.image, '[WARNING] No logo found', (int(self.width/5), 20), cv2.FONT_HERSHEY_DUPLEX, .5, (0, 0, 255))
            
            self.detection = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            self.publisher.publish(self.detection)

            self.video_data.append(self.image)

            rospy.logwarn('No logo found')
            return 0
        
        if cx > self.width/2 - tol and cx < self.width/2 + tol:
            return 1
        if cx < self.width/2 - tol:
            return 2
        if cx > self.width/2 + tol:
            return 3

if __name__ == '__main__':
    image_processor = LineDetector()
    # image = cv2.imread('/home/turtlepc/Documentos/project/img/0.png')
    # image_processor.test(image, 'black')
