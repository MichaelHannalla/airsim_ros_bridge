#! /usr/bin/env python

import numpy as np
import airsim 
import rospy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped

client = airsim.CarClient()
client.confirmConnection()

ros_gps = NavSatFix()
ros_gps_vel = Vector3Stamped()

ros_gps.status.service = ros_gps.status.SERVICE_GPS
ros_gps.header.frame_id = "odom"
ros_gps_vel.header.frame_id = "gps"

rospy.init_node("airsim_gps_node")
pos_pub = rospy.Publisher('airsim/gps/fix', NavSatFix, queue_size=10)
vel_pub = rospy.Publisher('airsim/gps/velocity', Vector3Stamped, queue_size=10)

r = rospy.Rate(10)

while not rospy.is_shutdown():
	airsim_gps = client.getGpsData(gps_name = "Gps", vehicle_name = "")

	ros_gps.altitude = -1*airsim_gps.gnss.geo_point.altitude
	ros_gps.longitude = -1*airsim_gps.gnss.geo_point.longitude 
	ros_gps.latitude = airsim_gps.gnss.geo_point.latitude
	
	ros_gps.header.stamp = rospy.Time.from_sec(airsim_gps.time_stamp / 1e9)
	ros_gps_vel.header.stamp = rospy.Time.from_sec(airsim_gps.time_stamp / 1e9)
	
	ros_gps_vel.vector.x = airsim_gps.gnss.velocity.x_val
	ros_gps_vel.vector.y = -1*airsim_gps.gnss.velocity.y_val
	ros_gps_vel.vector.z = -1*airsim_gps.gnss.velocity.z_val

	pos_pub.publish(ros_gps)
	vel_pub.publish(ros_gps_vel)

	r.sleep()


