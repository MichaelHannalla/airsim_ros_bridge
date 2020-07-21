#! /usr/bin/env python

import airsim
import rospy
import cv2
import numpy as np
import os
import time
import open3d as o3d
import matplotlib.pyplot as plt

import scipy
from scipy import interpolate
from scipy.spatial import Delaunay

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from std_msgs.msg import Float32

# Initializing ROS node
rospy.init_node("airsim_node")

# Initializing airsim client node and enabling API control
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()
car_controls.is_manual_gear = False

path_pub = rospy.Publisher('airsim/DelaunayPath', PoseArray, queue_size=1) 
ros_path = PoseArray()
ros_path.header.frame_id = "velodyne"

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

#inital values
x = 0.0
y = 0.0
vx = 0
vy = 0
vth = 0
current_time = rospy.Time.now()

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

# Defining subscriber callback functions
def throttle_callback(throttle_input):
	try:
		car_controls.throttle = float(throttle_input.data)
		client.setCarControls(car_controls)
		rospy.loginfo("Sent throttle to simulator.")
	except (RuntimeError, ValueError, BufferError, IOError, AssertionError):
		print("Exception happened while processing throttle.")
		pass


def steering_angle_callback(steering_angle_input):
	try:
		car_controls.steering = float(steering_angle_input.data)
		client.setCarControls(car_controls)
		rospy.loginfo("Sent steering angle to simulator.")
	except (RuntimeError, ValueError, BufferError, IOError, AssertionError):
		print("Exception happened while processing steering angle.")
		pass

rospy.Subscriber("SteeringAngleData", Float32, queue_size= 1, callback= steering_angle_callback)
rospy.Subscriber("SpeedData", Float32, queue_size= 1, callback= throttle_callback)

while not rospy.is_shutdown():
	try:
		try:
			current_time = rospy.Time.now()
			car_state = client.getCarState()
			x = car_state.kinematics_estimated.position.x_val
			y = -1 * car_state.kinematics_estimated.position.y_val
			current_car_controls = client.getCarControls()
			z_steering = current_car_controls.steering

			wo = car_state.kinematics_estimated.orientation.w_val
			xo = car_state.kinematics_estimated.orientation.x_val
			yo = car_state.kinematics_estimated.orientation.y_val
			zo = -1 * car_state.kinematics_estimated.orientation.z_val
			vx = car_state.kinematics_estimated.linear_velocity.x_val
			vy = -1 * car_state.kinematics_estimated.linear_velocity.y_val
			vth = car_state.kinematics_estimated.angular_velocity.z_val
			odom_quat = (xo, yo, zo, wo)
			odom_broadcaster.sendTransform(
			(x, y, 0),
			odom_quat,
			current_time,
			"base_link",
			"odom"
			)
			odom = Odometry()
			odom.header.stamp = current_time
			odom.header.frame_id = "odom"
			odom.pose.pose = Pose(Point(x, y, z_steering), Quaternion(*odom_quat))
			odom.child_frame_id = "base_link"
			odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
			odom_pub.publish(odom)
			rospy.loginfo("Published odometry message.")
		except (RuntimeError, ValueError, BufferError, IOError, AssertionError):
			print("Exception happened while processing state estimate.")
			pass
		

		try:
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
				path_pub.publish(ros_path)
				rospy.loginfo("Published path message.")

		except (RuntimeError, ValueError, BufferError, IOError, AssertionError):
			print("Exception happened while processing lidar and delaunay.")
			pass

	except KeyboardInterrupt:
		client.enableApiControl(False)
		print("Forcing shutdown.")
		exit()