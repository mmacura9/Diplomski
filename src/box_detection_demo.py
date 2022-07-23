#!/usr/bin/env python3
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import numpy as np
from std_msgs.msg import Float64MultiArray
import sys
	


       
def image_callback(data):
    rospy.loginfo("Received an image!")
    data = np.array([0.06533789520091933, 0.14741829399521938, 1.4797075733437641])
    pab.publish(Float64MultiArray(data = data))
    print(data)
                
def main():
    global pab
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/depth/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback, queue_size=1)
    pab = rospy.Publisher("/target_coordinates", Float64MultiArray, queue_size=0)
    # Spin until ctrl + c
    r = rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__':
    main()
