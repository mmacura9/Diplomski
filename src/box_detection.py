#!/usr/bin/env python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np

# Instantiate CvBridge
bridge = CvBridge()

def detect_box(depth_array: np.array):
    depth = np.copy(depth_array)
    #cv2.normalize(depth, depth, 0, 1, cv2.NORM_MINMAX)
    depth = np.floor(255*depth)
    #depth = np.stack([depth, depth, depth], axis = 2)
    depth = depth.astype(np.uint8)
    # prebacivanje u gray scale
    #imgray = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)
    # prebacivanje u binarnu sliku
    ret, thresh = cv2.threshold(depth, 100, 255, 0)
    rospy.loginfo(thresh.shape)
    # nalazenje kontura
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # nalazenje najvece konture po povrsini
    area = 0
    cnt_out = contours[0]
    for cnt in contours:
        ar1 = cv2.contourArea(cnt)
        if area < ar1:
            area = ar1
            cnt_out = cnt
    
    cv2.drawContours(depth, cnt_out, -1, (0,255,0), 3)
    cv2.imwrite('./src/diplomski/test_slike/depth_image.png', depth)

def image_callback(data):
    rospy.loginfo("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(data, "32FC1")
        depth_array = np.array(cv2_img, dtype=np.float32)
        depth_array[depth_array>1.5] = 0
        rospy.loginfo(depth_array.shape)
        rospy.loginfo(np.max(depth_array))
        
        detect_box(depth_array)
        
    except CvBridgeError as e:
            raise CvBridgeError(e)
        
def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/depth/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
