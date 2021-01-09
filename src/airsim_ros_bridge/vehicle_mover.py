#! /usr/bin/env python

import airsim 
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension


# Initializing airsim client node and enabling API control
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()
car_controls.is_manual_gear = False

# Initializing ROS node
rospy.init_node("airsim_vehicle_mover")
r = rospy.Rate(2.0)


# Defining subscriber callback functions
def throttle_callback(throttle_input):
	try:
		car_controls.throttle = float(throttle_input.data[0])
		client.setCarControls(car_controls)
		r.sleep()
	except (RuntimeError, ValueError, BufferError, IOError, AssertionError, TypeError, IndexError):
		pass
		#print("Catching exceptions")

def steering_angle_callback(steering_angle_input):
	#client=airsim.CarClient()
	try:
		car_controls.steering =  float(steering_angle_input.data[0])
		client.setCarControls(car_controls)
		r.sleep()
	except (RuntimeError, ValueError, BufferError, IOError, AssertionError, TypeError, IndexError):
		pass
		#print("Catching exceptions")

rospy.Subscriber('controls/AI2VCU_Steer', Float64MultiArray, queue_size= 1, callback= steering_angle_callback)
rospy.Subscriber('controls/AI2VCU_Drive_F', Float64MultiArray, queue_size= 1, callback= throttle_callback)

while not rospy.is_shutdown():
	try:
		pass
	except KeyboardInterrupt:
		client.enableApiControl(False)
		exit()
