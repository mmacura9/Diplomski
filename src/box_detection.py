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

def distance(P1: np.array, P2: np.array, P3: np.array) -> float:
    """


    Parameters
    ----------
    P1 : np.array
        Point 1.
    P2 : np.array
        Point 2.
    P3 : np.array
        Point 3.

    Returns
    -------
    float
        The distance between line (P1, P2) and point P3.

    """
    if np.all(P1 == P2):
        P = P3-P2
        return abs(P[0]-P[1])
    return np.linalg.norm(np.cross(np.array(P2)-np.array(P1), np.array(P1)-np.array(P3)))/np.linalg.norm(np.array(P2)-np.array(P1))
    
def max_distance(points: np.array, P1: np.array, P2: np.array) -> tuple:
    dist = 0
    ind = 0
    for i in range(1, len(points)):
        if dist < distance(P1, P2, points[i]):
            dist = distance(P1, P2, points[i])
            ind = i
    return ind, dist
    
def normala(P1: np.array, P2: np.array, P3: np.array) -> np.array:
    """
    

    Parameters
    ----------
    P1 : np.array
        Point 1.
    P2 : np.array
        Point 2.
    P3 : np.array
        Point 3.

    Returns
    -------
    np.array
        The point P in line (P1, P2) that line (P, P3) is ortoghonal to line (P1, P2)

    """
    line = [P2[0]-P1[0], P2[1]-P1[1]]
    if line[1] == 0:
        return np.array([-1, -1])
    if line[0] == 0:
        v = [1, 0]
    else:
        v = [1, -line[0]/line[1]]
    a1 = line[1]/line[0]
    b1 = P1[1] - a1*P1[0]
    
    a2 = v[1]/v[0]
    b2 = P3[1] - a2*P3[0]
    
    P4 = np.zeros(2)
    P4[0] = (b2-b1)/(a1-a2)
    P4[1] = a1*P4[0]+b1
    
    return P4
    
def granice(points: np.array, P1:np.array, P2: np.array, P3: np.array) -> np.array:
    P = normala(P1, P2, P3)
    if P[0] == -1 and P[1] == -1:
        return np.array(points)[:, 0] < P3[0]
    
    l = [P3[0] - P[0], P3[1] - P[1]]
    a = l[1]/l[0]
    b = P[1] - a*P[0]
    return np.array(points)[:, 1] <= a*np.array(points)[:, 0] + b
    
def split(points: np.array, threshold: float) -> tuple:
    lines = []
    if len(points) == 0:
        return lines
    if len(points) == 1:
        return [points]
    P1 = points[0]
    P2 = points[-1]
    ind_max, dist = max_distance(points, P1, P2)
    
    if dist < threshold:
        lines.append(points)
    else:
        P3 = points[ind_max]
        s1 = granice(points, P1, P2, P3)
        if np.sum(s1) == len(points):
            s1[ind_max] = False
        if np.sum(s1) == 0:
            s1[ind_max] = True
        lines = lines + split(np.array(points)[s1].tolist(), threshold)
        lines = lines + split(np.array(points)[np.logical_not(s1)].tolist(), threshold)
    return lines

def find4best(lines: list) -> tuple:
    ind_1 = 0
    ind_2 = 0
    ind_3 = 0
    ind_4 = 0
    max_len = 0
    for i in range(len(lines)):
        if max_len < len(lines[i]):
            max_len = len(lines[i])
            ind_1 = i
            
    max_len = 0
    for i in range(len(lines)):
        if max_len < len(lines[i]) and i != ind_1:
            max_len = len(lines[i])
            ind_2 = i
            
    max_len = 0
    for i in range(len(lines)):
        if max_len < len(lines[i]) and i != ind_1 and i != ind_2:
            max_len = len(lines[i])
            ind_3 = i
    
    max_len = 0
    for i in range(len(lines)):
        if max_len < len(lines[i]) and i != ind_1 and i != ind_2 and i != ind_3:
            max_len = len(lines[i])
            ind_4 = i
    
    return (ind_1, ind_2, ind_3, ind_4)

def merge(lines: list, threshold: float) -> list:
    i = 0
    num = -1
    while num != len(lines):
        num = len(lines)
        i = 0
        j = 0
        while i < len(lines):
            j = i + 1
            while j < len(lines):
                l = lines[i] + lines[j]
                if len(lines[i]) == 1:
                    dist = np.array(l[0]) - np.array(l[1])
                    dist = dist[0] + dist[1]
                    if dist < 10:
                        lines[i] = l
                        lines.pop(j)
                        continue
                a, b = np.polyfit(np.array(l)[:, 0], np.array(l)[:, 1], 1)
                P1 = [0, b]
                P2 = [1, a+b]
                ind, dist = max_distance(l, P1, P2)
                if dist < threshold:
                    lines[i] = l
                    lines.pop(j)
                else:
                    j = j + 1
            i = i + 1
    i = 0
    ind_1, ind_2, ind_3, ind_4 = find4best(lines)
    return [lines[ind_1], lines[ind_2], lines[ind_3], lines[ind_4]]

def split_and_merge(points: np.array) -> np.array:
    lines = split(points, 5)
    lines = merge(lines, 1)
    return lines

def box_detection(img: np.array, lines: list, a: list, b: list) -> np.array:
    mean_val = np.zeros([4, 2])
    up_limit = img.size
    down_limit = 0
    left_limit = img.size
    right_limit = 0
    up = 0
    down = 0
    left = 0
    right = 0
    for i in range(len(lines)):
        line = np.array(lines[i])
        mean_val[i, :] = np.mean(line, axis=0)
        if mean_val[i, 0] < up_limit:
            up_limit = mean_val[i, 0]
            up = i
        if mean_val[i, 0] > down_limit:
            down_limit = mean_val[i, 0]
            down = i
        if mean_val[i, 1] < left_limit:
            left_limit = mean_val[i, 1]
            left = i
        if mean_val[i, 1] > right_limit:
            right_limit = mean_val[i, 1]
            right = i
    
    P1j = (b[up]-b[left])/(a[left]-a[up])
    P1i = a[left]*P1j + b[left]
    P1 = [P1i, P1j]
    
    P2j = (b[up]-b[right])/(a[right]-a[up])
    P2i = a[right]*P2j + b[right]
    P2 = [P2i, P2j]
    
    P3j = (b[down]-b[left])/(a[left]-a[down])
    P3i = a[left]*P1j + b[left]
    P3 = [P3i, P3j]
    
    P4j = (b[down]-b[right])/(a[right]-a[down])
    P4i = a[right]*P2j + b[right]
    P4 = [P4i, P4j]
    img1 = np.zeros(img.shape)
    img1[int(P1i), int(P1j)] = 255
    img1[int(P2i), int(P2j)] = 255
    img1[int(P3i), int(P3j)] = 255
    img1[int(P4i), int(P4j)] = 255
    cv2.imwrite('./src/diplomski/test_slike/output.png', img1)
    return P1, P2, P3, P4

def main1(depth_array: np.array):
    depth = np.copy(depth_array)
    cv2.normalize(depth, depth, 0, 1, cv2.NORM_MINMAX)
    depth = np.floor(255*depth)
    depth = 255*(depth<99)
    depth = depth.astype(np.uint8)
    
    # prebacivanje u binarnu sliku
    ret, thresh = cv2.threshold(depth, 100, 255, 0)
    
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
    rect = cv2.minAreaRect(cnt_out)
    box = cv2.boxPoints(rect)
    print(box)
    box = np.int0(box)
    cv2.drawContours(depth, [box], 0, (0, 0, 255), 2)
    cv2.imwrite('./src/diplomski/test_slike/output.png', depth)
    #cnt_out = cnt_out[:, 0, :].tolist()
    #lines = split_and_merge(cnt_out)
    #a = []
    #b = []
    #for line in lines:
        #a1, b1 = np.polyfit(np.array(line)[:, 0], np.array(line)[:, 1], 1)
        #a = a + [a1]
        #b = b + [b1]
    #box = box_detection(depth, lines, a, b)

def image_callback(data):
    rospy.loginfo("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(data, "32FC1")
        depth_array = np.array(cv2_img, dtype=np.float32)
        main1(depth_array)
        
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
