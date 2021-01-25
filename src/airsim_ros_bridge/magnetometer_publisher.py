#! /usr/bin/env python3

import airsim
import rospy
import numpy as np
from sensor_msgs.msg import MagneticField

client = airsim.CarClient()
client.confirmConnection()

#Initializing ros node and publisher 
rospy.init_node('airsim_magnetometer_node')
magnet_pub = rospy.Publisher('airsim/MagneticField', MagneticField, queue_size=1)

#Initializing IMU message
ros_mag = MagneticField()
ros_mag.header.frame_id = "base_link"

r =rospy.Rate(200.0)

try:		
	while not rospy.is_shutdown():
		
		airsim_mag = client.getMagnetometerData(magnetometer_name = "", vehicle_name = "")

		# Preparing Magnetometer message
		ros_mag.header.stamp = rospy.Time.now()
		ros_mag.magnetic_field.x = airsim_mag.magnetic_field_body.x_val
		ros_mag.magnetic_field.y = -1.0 * airsim_mag.magnetic_field_body.y_val
		ros_mag.magnetic_field.z = -1.0 * airsim_mag.magnetic_field_body.z_val

		magnet_pub.publish(ros_mag)

		r.sleep()

except KeyboardInterrupt:
	print("Exited")
