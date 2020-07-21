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
rospy.init_node('airsim_tf_to_odom_node')
odom_pub = rospy.Publisher("odom_map", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()
listener = tf.TransformListener()
#inital values
x = 0.0
y = 0.0
vx = 0
vy = 0
vth = 0
r = rospy.Rate(10.0)

while not rospy.is_shutdown():
    try:
	#gmapping provides map to odom transform
	(trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	continue
    
    current_time = rospy.Time.now()
    car_state = client.getCarState()
    x = trans[0]
    y = trans[1]
    wo = rot[3]
    xo = rot[0]
    yo = rot[1]
    zo = rot[2]
    vx = car_state.kinematics_estimated.linear_velocity.x_val
    vy = -1*car_state.kinematics_estimated.linear_velocity.y_val
    vth = car_state.kinematics_estimated.angular_velocity.z_val
    odom_quat = (xo, yo, zo, wo)
    odom_broadcaster.sendTransform(
	(x, y, 0.),
	odom_quat,
	current_time,
	"base_link",
	"odom"
    )
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom_map"
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    odom_pub.publish(odom)
    r.sleep()
