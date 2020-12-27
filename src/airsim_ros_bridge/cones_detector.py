#! /usr/bin/env python

import airsim
import rospy
import cv2
import numpy as np
import os
import time
#import open3d as o3d
import matplotlib.pyplot as plt

from scipy import interpolate

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

client = airsim.CarClient()
client.confirmConnection()

#Initializing ros node and publishers. 
rospy.init_node('airsim_cones_detector_node')
right_cones_pub = rospy.Publisher('airsim/RightCones', PoseArray, queue_size=10) 
left_cones_pub = rospy.Publisher('airsim/LeftCones', PoseArray, queue_size=10)
ros_right_cones = PoseArray()
ros_left_cones = PoseArray()
ros_right_cones.header.frame_id = "velodyne"
ros_left_cones.header.frame_id = "velodyne"
temp_cone = Pose()

r = rospy.Rate(5)

# Defining the clustering function.
def radius_nms_np(boxes,radius):
  final_cones_idx = np.arange(0,boxes.shape[0])
  num_boxes = boxes.shape[0]
  
  for bi in range(num_boxes):
    if bi >= final_cones_idx.shape[0]:
      break
    b1_idx = final_cones_idx[bi]
    b = boxes[b1_idx].reshape(1,2)
    diff = boxes[final_cones_idx]-b
    diff_sq = np.sum(diff*diff,axis=1)
    dist = np.sqrt(diff_sq)
    final_cones_idx = final_cones_idx[np.where(dist>radius)]
    arr_idx = np.array([b1_idx],dtype=np.int)
    final_cones_idx = np.append(final_cones_idx,arr_idx,axis=0)
  return final_cones_idx


while not rospy.is_shutdown():
  # Getting lidar data.
  num_lidar_channels = 64
  airsim_lidar = client.getLidarData(lidar_name="Lidar", vehicle_name = "")
  point_data = np.asarray(airsim_lidar.point_cloud, np.float32)

  if len(point_data) >= 3:
    point_data = point_data.reshape((int(point_data.shape[0]/5),5))

    # Temporary solution of cone detection using lidar pointcloud height thresholding.
    point_data_x = point_data[:,0]
    point_data_y = point_data[:,1]
    point_data_z = point_data[:,2]
    cond = point_data_z < 0
    point_data_x = point_data_x[cond]
    point_data_y = point_data_y[cond]
    point_data_z = point_data_z[cond]
    dist = point_data_x*point_data_x + point_data_y*point_data_y + point_data_z*point_data_z
    dist = np.sqrt(dist)
    dist = dist<20
    point_data_x = point_data_x[dist]
    point_data_y = -1 * point_data_y[dist]                 # inversion of airsim's y-coordinate
    point_data_z = point_data_z[dist]
    point_cloud = np.array([point_data_x,point_data_y]).transpose()

  # NMS
  box_indices = radius_nms_np(point_cloud,2)
  center_points = point_cloud[box_indices]			   # Contains the x and y coordinates of the cones
  
  # Michael's edits start at this line.
  # Edited to classify between right and left points using spline fitting algorithms.
  # Publishes ROS topics of type PoseArray.
      
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
