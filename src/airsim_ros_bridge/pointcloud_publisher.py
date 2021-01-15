#! /usr/bin/env python

import airsim
import rospy
import ros_numpy
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

#import open3d as o3d
#pcd = o3d.geometry.PointCloud()

client = airsim.CarClient()
client.confirmConnection()

#Initializing ros node and publisher 
rospy.init_node('airsim_pointcloud_node')
lidar_pub = rospy.Publisher('airsim/VelodynePoints', PointCloud2, queue_size=10)

#Initializing pointcloud message
ros_lidar = None
r =rospy.Rate(10.0)

try:		
	while not rospy.is_shutdown():
		
		#Getting real time data from AirSim Server
		if not client.simIsPause():
			airsim_lidar = client.getLidarData(lidar_name="Lidar", vehicle_name = "")
			point_data = np.array(airsim_lidar.point_cloud, dtype=np.dtype('f4'))

		#Current scan message is inconsistent, latch previous message.
		if len(point_data)<3: 
			lidar_pub.publish(ros_lidar)

		else:
		    #Reshaping of point data to (num_points, 4)
			try: 
				point_data = np.reshape(point_data, (int(point_data.shape[0]/4), 4))
		
				point_prepared = np.zeros(len(point_data), dtype=[('x', 'f4'), ('y', 'f4'),('z', 'f4'), ('intensity', 'f4')])
				point_prepared['x'] = point_data[:,0]
				point_prepared['y'] = -1 * point_data[:,1]
				point_prepared['z'] = -1 * point_data[:,2]
				point_prepared['intensity'] = point_data[:,3]
			except (ValueError, IndexError):
				point_data = np.reshape(point_data, (int(point_data.shape[0]/5), 5))
		
				point_prepared = np.zeros(len(point_data), dtype=[('x', 'f4'), ('y', 'f4'),('z', 'f4')])
				point_prepared['x'] = point_data[:,0]
				point_prepared['y'] = -1 * point_data[:,1]
				point_prepared['z'] = -1 * point_data[:,2]
		
			#Preparing LiDAR message
			ros_lidar = ros_numpy.point_cloud2.array_to_pointcloud2(point_prepared)

			#Publishing the ROS messages with stamps and frame identities
			ros_lidar.header.frame_id = "velodyne"
			ros_lidar.header.stamp = rospy.Time.now()

			lidar_pub.publish(ros_lidar)
			
		r.sleep()

except KeyboardInterrupt:
	print("Exited")
