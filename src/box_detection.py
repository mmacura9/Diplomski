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

def image_callback(data):
    rospy.loginfo("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(data, "32FC1")
        depth_array = np.array(cv2_img, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        cv2.imwrite('./src/diplomski/test_slike/depth_image.png', np.floor(depth_array*256))
        rospy.loginfo(np.max(depth_array))
        rospy.loginfo(np.min(depth_array))
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