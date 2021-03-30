#! /usr/bin/env python

import airsim
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

import numpy as np

# Initializing airsim client node and enabling API control
global global_throttle
global client
global car_controls
global_throttle = 0
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()
car_controls.is_manual_gear = False

# Initializing ROS node
rospy.init_node("airsim_vehicle_mover")
r = rospy.Rate(10)

# Defining subscriber callback functions
def throttle_callback(throttle_input):
	global client
	global global_throttle
	try:
		global_throttle = float(throttle_input.data[0])
	except (RuntimeError, ValueError, BufferError, IOError, AssertionError, TypeError, IndexError):
		pass

def steering_angle_callback(steering_angle_input):
	global client
	global global_throttle
	global car_controls
	try:
		car_controls.throttle = global_throttle
		car_controls.steering =  -1 * float(steering_angle_input.data[0]) * 2.0
		client.setCarControls(car_controls)
	except (RuntimeError, ValueError, BufferError, IOError, AssertionError, TypeError, IndexError):
		pass

rospy.Subscriber('controls/AI2VCU_Steer', Float64MultiArray, queue_size= 1, callback= steering_angle_callback)
rospy.Subscriber('controls/AI2VCU_Drive_F', Float64MultiArray, queue_size= 1, callback= throttle_callback)

while not rospy.is_shutdown():
	try:
		pass
	except KeyboardInterrupt:
		client.enableApiControl(False)
		exit()
