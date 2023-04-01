#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

def read_rgb_image(image_name, show):
    rgb_image = cv2.imread(image_name)
    if show: 
        cv2.imshow("RGB Image",rgb_image)
    return rgb_image


def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image",hsv_image)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask


def getContours(binary_image):
    kernel = np.ones((5,5),np.uint8)
    binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel) 
    _, contours, hierarchy = cv2.findContours(binary_image, 
                                             cv2.RETR_TREE, 
                                              cv2.CHAIN_APPROX_SIMPLE)
    # contours, hierarchy = cv2.findContours(binary_image.copy(), 
    #                                         cv2.RETR_EXTERNAL,
    #                                         cv2.CHAIN_APPROX_SIMPLE)
    return contours


def draw_ball_contour(binary_image, rgb_image, contours):
    # black_image = np.zeros([binary_image.shape[0], binary_image.shape[1]], np.uint8)
    black_image = np.zeros_like(binary_image, np.uint8)
    if len(contours) > 0:
        c = sorted(contours, key=cv2.contourArea, reverse=True)[0]
        c = cv2.convexHull(c)
        # for c in contours:
        #     area = cv2.contourArea(c)
        #     perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        #     if (area>1000):

        # cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
        cv2.drawContours(black_image, [c], 0, (255,255,255), -1)
        cx, cy = get_contour_center(c)
        # cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
        cv2.circle(black_image, (cx,cy),5,(0,0,0),-1)

                #print ("Area: {}, Perimeter: {}".format(area, perimeter))
        #print ("number of contours: {}".format(len(contours)))
        cv2.imshow("RGB Image Contours",rgb_image)
        cv2.imshow("Black Image Contours",black_image)


def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def detect_ball_in_a_frame(image_frame):
    yellowLower = np.array([4, 110, 106], np.uint8)
    yellowUpper = np.array([21, 253, 215], np.uint8)
    rgb_image = image_frame.copy()
    binary_image_mask = filter_color(rgb_image, yellowLower, yellowUpper)
    contours = getContours(binary_image_mask)
    draw_ball_contour(binary_image_mask, rgb_image,contours)

def image_callback(ros_image):
  print ('got an image')
  global bridge
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  #from now on, you can work exactly like with opencv
  detect_ball_in_a_frame(cv_image)
  cv2.waitKey(1)   

  
def main(args):
  rospy.init_node('image_converter', anonymous=True)
  image_sub = rospy.Subscriber("/bebop/image_raw",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")