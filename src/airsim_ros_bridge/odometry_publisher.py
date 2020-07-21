#! /usr/bin/env python

import airsim 
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#airsim
client = airsim.CarClient()
client.confirmConnection()
#ros
rospy.init_node('airsim_odometry_publisher_node')
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()
#inital values
x = 0.0
y = 0.0
vx = 0
vy = 0
vth = 0
current_time = rospy.Time.now()

r = rospy.Rate(5.0)

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
    vy = -1 * car_state.kinematics_estimated.linear_velocity.y_val
    vth = car_state.kinematics_estimated.angular_velocity.z_val
    odom_quat = (xo, yo, zo, wo)
    odom_broadcaster.sendTransform(
	(x, y, 0),
	odom_quat,
	current_time,
	"base_link",
	"odom"
    )
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(x, y, z_steering), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    odom_pub.publish(odom)
    r.sleep()
