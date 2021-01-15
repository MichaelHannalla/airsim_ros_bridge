#! /usr/bin/env python

import airsim
import rospy
import numpy as np
from sensor_msgs.msg import Imu

client = airsim.CarClient()
client.confirmConnection()

#Initializing ros node and publisher 
rospy.init_node('airsim_imu_node')
imu_pub = rospy.Publisher('airsim/Imu', Imu, queue_size=1)

#Initializing IMU message
ros_imu = Imu()
r =rospy.Rate(200.0)

try:		
	while not rospy.is_shutdown():
		
		airsim_imu = client.getImuData(imu_name = "Imu", vehicle_name = "")

		#Preparing IMU message
		ros_imu.angular_velocity.x = airsim_imu.angular_velocity.x_val
		ros_imu.angular_velocity.y = airsim_imu.angular_velocity.y_val
		ros_imu.angular_velocity.z = -1 * airsim_imu.angular_velocity.z_val
		ros_imu.linear_acceleration.x = airsim_imu.linear_acceleration.x_val
		ros_imu.linear_acceleration.y = -1 * airsim_imu.linear_acceleration.y_val
		ros_imu.linear_acceleration.z = -1 * airsim_imu.linear_acceleration.z_val
		ros_imu.orientation.w = airsim_imu.orientation.w_val
		ros_imu.orientation.x = airsim_imu.orientation.x_val
		ros_imu.orientation.y = airsim_imu.orientation.y_val
		ros_imu.orientation.z = -1 * airsim_imu.orientation.z_val
		
		#Publishing the ROS messages with stamps and frame identities
		ros_imu.header.frame_id = "imu_link"
		ros_imu.header.stamp = rospy.Time.now()

		imu_pub.publish(ros_imu)

		r.sleep()

except KeyboardInterrupt:
	print("Exited")
