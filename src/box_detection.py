#!/usr/bin/env python3
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np
import math
from std_msgs.msg import Float64MultiArray
import sys

# Instantiate CvBridge
bridge = CvBridge()
grid = np.zeros([20, 20])
pab = 0

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

def point_order(box: list) -> tuple:
    box = np.array(box)
    avg = np.mean(box, axis = 0)
    p1 = np.zeros(2)
    p2 = np.zeros(2)
    p3 = np.zeros(2)
    p4 = np.zeros(2)
    for point in box:
        if point[0]<=avg[0] and point[1]>=avg[1]:
            p1 = point
        if point[0]<=avg[0] and point[1]<=avg[1]:
            p2 = point
        if point[0]>=avg[0] and point[1]<=avg[1]:
            p3 = point
        if point[0]>=avg[0] and point[1]>=avg[1]:
            p4 = point
    return p1, p2, p3, p4

def min_cell_depth(img: np.array, grid_start: np.array, i: int, j: int, vx: int, vy: int):
    minimum = np.max(img)
    ind1 = vx[0]
    ind2 = vy[1]
    vx = vx/vx[0]
    vy = vy/vy[1]
    for a in range(int(ind1)):
        for b in range(int(ind2)):
            x = grid_start[i, j] + a* vx + b*vy
            y = img[int(x[1]), int(x[0])]
            if y < minimum and y != 0:
                minimum = y
    return minimum

def gaussian_func(M: float, sigma: float, x: float) -> float:
    const = 1/math.sqrt(2*math.pi*sigma**2)
    e = math.exp(-0.5*(x-M)**2/sigma**2)
    return const*e

def update_grid(grid: np.array, img: np.array, grid_start: np.array, vx: np.array, vy: np.array) -> np.array:
    for i in range(20):
        for j in range(20):
            min_depth = min_cell_depth(img, grid_start, i, j, vx, vy)
            p = gaussian_func(100, 2.5, min_depth) / (1.1* gaussian_func(100, 2.5, 100))
            grid[i, j] = grid[i, j] + math.log(p/(1 - p))
    return grid

def empty_grid(goal_grid: np.array) -> bool:
    return not np.any(goal_grid>0.2)

def deg2rad(x: float) -> float:
    return x*math.pi/180

def placing_algorithm(grid: np.array, d: float, grid_start: np.array, depth_array: np.array, depth) -> list:
    rows = 20
    columns = 20
    D = math.ceil(d/(0.5/20)) # 0.5m je velicina kutije, pa je Grid_resolution = 0.5/20
    print(D)
    for i in range(rows-D):
        for j in range(columns-D):
            goal_grid = grid[i: i+D, j: j+D]
            if empty_grid(goal_grid):
                x = grid_start[i + D//2, j + D//2, 0]
                y = grid_start[i + D//2, j + D//2, 1]
                sredina = np.array(depth_array.shape)//2
                alpha = math.atan(math.sqrt(math.tan((x-sredina[0])*1.047/640)**2+math.tan((y-sredina[1])*1.047/640)**2))
                distance = depth_array[int(y), int(x)]
                
                img1 = np.zeros([depth.shape[0], depth.shape[1], 3])
                img1[:, :, 0] = depth
                img1[:, :, 1] = depth
                img1[:, :, 2] = depth
                
                img1[int(y), int(x), 0] = 255
                img1[int(y), int(x), 1] = 0
                img1[int(y), int(x), 2] = 0
                
                cv2.imwrite('./src/diplomski/test_slike/mesto.png', img1)
                print(x, y, sredina, alpha, distance)
                print(depth_array[sredina[0], sredina[1]])
                z_coordinate = distance*math.cos(alpha)
                x_coordinate = -z_coordinate*math.sin((y-sredina[0])*1.047/640)
                y_coordinate = -z_coordinate*math.sin((x-sredina[1])*1.047/640)
                return [x_coordinate, y_coordinate, z_coordinate]

def main1(depth_array: np.array):
    global grid
    global pab
    d = 0.05
    gripper_size = 0.05
    depth = np.copy(depth_array)
    print(depth.shape)
    cv2.normalize(depth, depth, 0, 1, cv2.NORM_MINMAX)
    depth = np.floor(255*depth)
    
    #imgray = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)
    depth = depth.astype(np.uint8)
    ret, thresh = cv2.threshold(depth, 90, 255, 0)
    thresh = 255 - thresh
    
    # nalazenje kontura
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # nalazenje najvece konture po povrsini
    area = 0
    cnt_out = contours[0]
    cnt_ind = 0
    for i in range(len(contours)):
        ar1 = cv2.contourArea(contours[i])
        if area < ar1:
            area = ar1
            cnt_out = contours[i]
            cnt_ind = i
    
    area = 0
    # nalazenje najvece konture unutar najvece konture na slici
    for i in range(len(contours)):
        if hierarchy[0, i, 3] == cnt_ind:
            ar1 = cv2.contourArea(contours[i])
            if area < ar1:
                area = ar1
                cnt_out = contours[i]
                
    rect = cv2.minAreaRect(cnt_out)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(depth, [box], 0, (0, 0, 255), 2)
    cv2.imwrite('./src/diplomski/test_slike/output.png', depth)
    
    p3, p1, p2, _ = point_order(box)
    
    v1 = (p2-p1)/22
    v2 = (p3-p1)/22
    tackex = []
    tackey =[]
    p11 = np.copy(p1)
    p12 = np.copy(p1)
    for i in range(22):
        tackex.append(p11)
        tackey.append(p12)
        p11 = p11 + v1
        p12 = p12 + v2
        
    grid_start = np.zeros([22, 22, 2], dtype='int')
    for i in range(22):
        for j in range(22):
            grid_start[i, j, :] = np.floor(tackex[i]+v2*j)
    grid_start = grid_start[2:, 2:, :]
    img1 = np.zeros([depth.shape[0], depth.shape[1], 3])
    img1[:, :, 0] = depth
    img1[:, :, 1] = depth
    img1[:, :, 2] = depth

    for i in range(20):
        for j in range(20):
            img1[grid_start[i, j, 1], grid_start[i, j, 0], 0] = 255
            img1[grid_start[i, j, 1], grid_start[i, j, 0], 2] = 0
            img1[grid_start[i, j, 1], grid_start[i, j, 0], 1] = 0
    cv2.imwrite('./src/diplomski/test_slike/grid.png', img1)
    
    grid = update_grid(grid, depth, grid_start, v1, v2)
    bel = 1 - 1/(1-np.exp(grid))
    img1 = np.zeros([400, 400])
    for i in range(20):
        for j in range(20):
            img1[20*i:20*(i+1), 20*j:20*(j+1)] = int(bel[i, j]*255)
    cv2.imwrite('./src/diplomski/test_slike/bel.png', img1)
    pab.publish(Float64MultiArray(data = np.array(placing_algorithm(grid, d + gripper_size, grid_start, depth_array, depth))))
    print(placing_algorithm(grid, d + gripper_size, grid_start, depth_array, depth))

       
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
    global pab
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/depth/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    pab = rospy.Publisher("/target_coordinates", Float64MultiArray, queue_size=1)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
