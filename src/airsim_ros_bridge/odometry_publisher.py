#! /usr/bin/env python

import airsim 
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import numpy as np

#airsim
client = airsim.CarClient()
client.confirmConnection()

#ros
rospy.init_node('airsim_odometry_publisher_node')
odom_pub = rospy.Publisher("airsim/KinematicOdometry", Odometry, queue_size=1)
odom_local_pub = rospy.Publisher("airsim/KinematicOdometryLocal", Odometry, queue_size=1)
odom_broadcaster = tf.TransformBroadcaster()

#inital values
x = 0.0
y = 0.0
vx = 0
vy = 0
vth = 0
current_time = rospy.Time.now()

r = rospy.Rate(10.0)

odom = Odometry()
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"

odom_local = Odometry()
odom_local.header.frame_id = "base_link"
odom_local.child_frame_id = "base_link"

while not rospy.is_shutdown():
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
    vy  = -1 * car_state.kinematics_estimated.linear_velocity.y_val
    vth = -1 * car_state.kinematics_estimated.angular_velocity.z_val
    
    odom_quat = (xo, yo, zo, wo)
    odom_eul  = tf.transformations.euler_from_quaternion(odom_quat)
    yaw = odom_eul[2]

    '''
    odom_broadcaster.sendTransform(
	(x, y, 0),
	odom_quat,
	current_time,
	"base_link",
	"odom"
    )
    '''

    odom.header.stamp = current_time
    odom.pose.pose = Pose(Point(x, y, z_steering), Quaternion(*odom_quat))
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    odom_pub.publish(odom)


    # Local odometry feature
    transformation_matrix = np.mat([[np.cos(yaw), -1 * np.sin(yaw), 0],
                                    [np.sin(yaw), np.cos(yaw)     , 0],
                                    [0          , 0               , 1]])

    vel_local = np.linalg.inv(transformation_matrix) @ np.mat([[vx],[vy],[1]])

    odom_local.twist.twist.linear.x  = vel_local[0, 0]
    odom_local.twist.twist.linear.y  = vel_local[1, 0]
    odom_local.twist.twist.angular.z = vth

    odom_local_pub.publish(odom_local)
    r.sleep()
