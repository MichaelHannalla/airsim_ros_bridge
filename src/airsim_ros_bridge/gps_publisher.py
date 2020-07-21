#! /usr/bin/env python

import numpy as np
import airsim 
import rospy

from sensor_msgs.msg import NavSatFix

client = airsim.CarClient()
client.confirmConnection()

ros_gps = NavSatFix()
ros_gps.status.service = ros_gps.status.SERVICE_GPS
ros_gps.header.frame_id = "odom" 

rospy.init_node("airsim_gps_node")
pub = rospy.Publisher('airsim/gps/fix', NavSatFix, queue_size=10)

while not rospy.is_shutdown():
	airsim_gps = client.getGpsData(gps_name = "Gps", vehicle_name = "")
	ros_gps.altitude = airsim_gps.gnss.geo_point.altitude
	ros_gps.longitude = airsim_gps.gnss.geo_point.longitude 
	ros_gps.latitude = airsim_gps.gnss.geo_point.latitude
	ros_gps.header.stamp = rospy.Time.now()
	pub.publish(ros_gps)


