#! /usr/bin/env python

import airsim
import rospy
import time
import numpy as np
from rosgraph_msgs.msg import Clock

client = airsim.CarClient()
client.confirmConnection()

#Initializing ros node and publisher 
rospy.init_node('clock_node')
clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)

#Initializing IMU message
clock = Clock()

try:		
	while not rospy.is_shutdown():
		clock = rospy.Time.from_sec(client.getCarState().timestamp / 1e9)
		clock_pub.publish(clock)
		time.sleep(1/1000)
		
except KeyboardInterrupt:
	print("Exited")
