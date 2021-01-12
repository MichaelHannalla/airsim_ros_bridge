#! /usr/bin/env python

import airsim
import rospy
import ros_numpy
import cv2
import numpy as np
import os
import time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

#import open3d as o3d
#pcd = o3d.geometry.PointCloud()

client = airsim.CarClient()
client.confirmConnection()

#This is to enable controlling the vehicle from APIs 
#client.enableApiControl(True)

#Initializing ros node and publisher 
rospy.init_node('airsim_pointcloud_node')
imu_pub = rospy.Publisher('airsim/imu', Imu, queue_size=10)
lidar_pub = rospy.Publisher('airsim/VelodynePoints', PointCloud2, queue_size=10)

#Initializing IMU message
ros_imu = Imu()
ros_lidar = None
r =rospy.Rate(10.0)

try:		
	while not rospy.is_shutdown():
		#Getting real time data from AirSim Server
		if not client.simIsPause():
			airsim_imu = client.getImuData(imu_name = "Imu", vehicle_name = "")
			airsim_lidar = client.getLidarData(lidar_name="Lidar", vehicle_name = "")
			point_data = np.array(airsim_lidar.point_cloud, dtype=np.dtype('f4'))

		#Current scan message is inconsistent, latch previous message.
		if len(point_data)<3: 
			lidar_pub.publish(ros_lidar)

		else:
	
		    #Reshaping of point data to (num_points, 3) 
		    point_data = np.reshape(point_data, (int(point_data.shape[0]/4), 4))
	
		    point_prepared = np.zeros(len(point_data), dtype=[('x', 'f4'), ('y', 'f4'),('z', 'f4'), ('intensity', 'f4')])
		    point_prepared['x'] = point_data[:,0]
		    point_prepared['y'] = -1 * point_data[:,1]
		    point_prepared['z'] = -1 * point_data[:,2]
		    point_prepared['intensity'] = -1 * point_data[:,3]
	
		    #Visualization of point cloud output from AirSim API
		    #pcd.points = o3d.utility.Vector3dVector(point_data) # xyz is a numpy array with shape (num_points,3)
		    #o3d.visualization.draw_geometries([pcd])
	 
		    #Preparing IMU message
		    ros_imu.angular_velocity.x = airsim_imu.angular_velocity.x_val
		    ros_imu.angular_velocity.y = airsim_imu.angular_velocity.y_val
		    ros_imu.angular_velocity.z = airsim_imu.angular_velocity.z_val
		    ros_imu.linear_acceleration.x = airsim_imu.linear_acceleration.x_val
		    ros_imu.linear_acceleration.y = airsim_imu.linear_acceleration.y_val
		    ros_imu.linear_acceleration.z = airsim_imu.linear_acceleration.z_val
		    ros_imu.orientation.w = airsim_imu.orientation.w_val
		    ros_imu.orientation.x = airsim_imu.orientation.x_val
		    ros_imu.orientation.y = airsim_imu.orientation.y_val
		    ros_imu.orientation.z = airsim_imu.orientation.z_val
	
		    #Preparing LiDAR message
		    ros_lidar = ros_numpy.point_cloud2.array_to_pointcloud2(point_prepared)
	
		    #Publishing the ROS messages with stamps and frame identities
		    ros_imu.header.frame_id = "odom"
		    ros_imu.header.stamp = rospy.Time.now() 
		    ros_lidar.header.frame_id = "velodyne"
		    ros_lidar.header.stamp = rospy.Time.now()
	
		    imu_pub.publish(ros_imu)
		    lidar_pub.publish(ros_lidar)
			
		r.sleep()

except KeyboardInterrupt:
	print("Exited")
