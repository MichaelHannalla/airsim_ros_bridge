#! /usr/bin/env python

import airsim
import rospy
import cv2
import numpy as np
import os
import time
#import open3d as o3d
import matplotlib.pyplot as plt

import scipy
from scipy import interpolate
from scipy.spatial import Delaunay

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

#Initializing ros node and publishers. 
rospy.init_node('airsim_delaunay_planner_node')
path_pub = rospy.Publisher('airsim/DelaunayPath', PoseArray, queue_size=1) 
ros_path = PoseArray()
ros_path.header.frame_id = "velodyne"

client = airsim.CarClient()
client.confirmConnection()
car_controls = airsim.CarControls()

r = rospy.Rate(0.5)
laps = 0
moved_from_centre = False

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

  if len(point_data) >= 3: # Checking airsim's lidar output integrity
    point_data = point_data.reshape(int(point_data.shape[0]/5),5)

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
    dist = dist < 10
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
        
    #ind = np.argsort( center_points[:,0] )
    #center_points = center_points[ind]                     # Sorting points from near to far, based on x-coordinate
    #center_points = np.vstack(([0,0], center_points))     # Adding the lidar main point as a point to be used in spline fitting
    #center_points = np.hstack((center_points, np.zeros((len(center_points),1))))
    
    try:
      triangulation = Delaunay(center_points, qhull_options='QJ')
    except (scipy.spatial.qhull.QhullError, ValueError):
      continue

    midpoints = []
    midpoints = np.zeros((1,2), dtype=np.float)

    for idx in range(len(triangulation.simplices)):
      arr1 = center_points[triangulation.simplices[idx,:]]
      arr2 = [arr1[-1], arr1[0], arr1[1]]
      midpoints = np.vstack((midpoints, (arr1 + arr2) / 2))
    midpoints = np.unique(midpoints, axis = 0)

    poly_coeff = np.polyfit(center_points[:,0], center_points[:,1], deg=2)
    polyout = np.polyval(poly_coeff,center_points[:,0])

    right_cones = (polyout > center_points[:,1])
    right_cones = center_points[right_cones]
    left_cones = (polyout <= center_points[:,1])
    left_cones = center_points[left_cones]

    right_poly_coeff = np.polyfit(right_cones[:,0], right_cones[:,1], deg=1)
    left_poly_coeff = np.polyfit(left_cones[:,0], left_cones[:,1], deg=1)
    right_polyout = np.polyval(right_poly_coeff,midpoints[:,0])
    left_polyout = np.polyval(left_poly_coeff,midpoints[:,0])

    path = np.logical_and(right_polyout < np.subtract(midpoints[:,1],1), left_polyout > np.add(midpoints[:,1],1))
    path = midpoints[path]
    ind = np.argsort(path[:,0])
    path = path[ind]
    
    ros_path.poses = []

    for point in path:
      point = np.append(point, 0)
      ros_path.poses.append(Pose(Point(*point), Quaternion(0,0,0,0)))

    ros_path.header.stamp = rospy.Time.now()

    car_state = client.getCarState()
    x = car_state.kinematics_estimated.position.x_val
    y = car_state.kinematics_estimated.position.y_val

    displacement_from_origin = np.sqrt(x**2 + y**2)

    if displacement_from_origin > 1.0 and (not moved_from_centre):
      moved_from_centre = True
    
    if moved_from_centre and displacement_from_origin < 1.0:
      moved_from_centre = False
      rospy.loginfo("Finished one race lap.")
      laps += 1

    if laps >= 1:
       ros_path.poses = []
       path_pub.publish(ros_path)
       car_controls.throttle = 0
       car_controls.steering = 0
       client.setCarControls(car_controls)
       client.enableApiControl(False)
       rospy.loginfo("Race finished.")
       exit()

    path_pub.publish(ros_path)
    r.sleep()
    
 
