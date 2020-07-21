#! /usr/bin/env python

import cv2
import rospy
import airsim
import matplotlib.pyplot as plt 
import numpy as np

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

client = airsim.CarClient()
client.confirmConnection()

focal_length = 128

#Initializing ros node and publishers. 
rospy.init_node('visual_airsim_cones_detector_node')
right_cones_pub = rospy.Publisher('airsim/RightCones', PoseArray, queue_size=10) 
left_cones_pub = rospy.Publisher('airsim/LeftCones', PoseArray, queue_size=10)
ros_right_cones = PoseArray()
ros_left_cones = PoseArray()
ros_right_cones.header.frame_id = "velodyne"
ros_left_cones.header.frame_id = "velodyne"
temp_cone = Pose()
r = rospy.Rate(2)

while not rospy.is_shutdown():
    responses = client.simGetImages([ airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False), 
        airsim.ImageRequest("0", airsim.ImageType.DepthPlanner, True, False)]) 

    img1d = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8) #get numpy array
    img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)  #reshape array to 3 channel image array H X W X 3
    depth_map = responses[1].image_data_float
    depth_map = np.reshape(depth_map, (responses[1].height, responses[1].width))

    hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)

    l_h, l_s, l_v = 90, 138, 240
    u_h, u_s, u_v = 179, 255, 255

    lower_ = np.array([l_h, l_s, l_v])
    upper_ = np.array([u_h, u_s, u_v])
    thresholded = cv2.inRange(hsv, lower_, upper_)
    
    ## [findContours]
    # Find contours
    _, contours, _ = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    ## [findContours]

    # Get the moments
    mu = [None]*len(contours) #Creating a list of Nulls with length of Contours list
    for i in range(len(contours)):
        mu[i] = cv2.moments(contours[i])

    # Get the mass centers (Centroid)
    mc = [None]*len(contours)
    for i in range(len(contours)):
        # add 1e-5 to avoid division by zero
        mc[i] = (mu[i]['m10'] / (mu[i]['m00'] + 1e-5), mu[i]['m01'] / (mu[i]['m00'] + 1e-5))
    res = cv2.bitwise_and(img_rgb,img_rgb, mask= thresholded)

    for i in range(len(contours)):
        color = (0, 0, 0)
        cv2.drawContours(img_rgb, contours, i, color, 2)
        cv2.circle(img_rgb, (int(mc[i][0]), int(mc[i][1])), 4, color, -1)

    cone_detections_pixels = np.asarray(np.round(mc), dtype=np.uint8)
    cone_detections = []

    for pixel_idx in cone_detections_pixels:
        row_pixel, column_pixel = int(pixel_idx[1]), int(pixel_idx[0])
        if row_pixel != 0 and column_pixel != 0:
            current_depth = depth_map[row_pixel, column_pixel]
            current_y = (-column_pixel + 72) * current_depth / focal_length
            cone_detections.append([current_depth, current_y])

    cone_detections = np.asarray(cone_detections)
    accepted_depth = cone_detections[:,0] <= 10
    cone_detections = cone_detections[accepted_depth]
    
    #plt.plot(cone_detections[:,0], cone_detections[:,1], 'o')
    #plt.show()
    
    
    #cv2.imshow('Raw Image',img_rgb)
    #cv2.imshow('Thresholding Result',res)
    #cv2.waitKey(1)

    center_points = np.asarray(cone_detections)
    ind = np.argsort( center_points[:,0] )
    center_points = center_points[ind]                     # Sorting points from near to far, based on x-coordinate
    #center_points = np.vstack(([0,0], center_points))     # Adding the lidar main point as a point to be used in spline fitting
    #center_points = np.hstack((center_points, np.zeros((len(center_points),1))))
        
    # Checking integrity of the number of detected cones.
    if len(center_points) >= 4:
    
        # Using numpy polynomial fitting.
        poly_coeff = np.polyfit(center_points[:,0], center_points[:,1], deg=2)
        polyout = np.polyval(poly_coeff,center_points[:,0])

        right_cones = (polyout > center_points[:,1])
        right_cones = center_points[right_cones]
        left_cones = (polyout <= center_points[:,1])
        left_cones = center_points[left_cones]
        ros_right_cones.poses = []
        ros_left_cones.poses = []
        
        for point in right_cones:
            point = np.append(point, 0)
            ros_right_cones.poses.append(Pose(Point(*point), Quaternion(0,0,0,0)))

        for point in left_cones:
            point = np.append(point, 0)
            ros_left_cones.poses.append(Pose(Point(*point), Quaternion(0,0,0,0)))
        
    ros_right_cones.header.stamp = rospy.Time.now()
    ros_left_cones.header.stamp = ros_right_cones.header.stamp
    
    right_cones_pub.publish(ros_right_cones)
    left_cones_pub.publish(ros_left_cones)
    r.sleep()