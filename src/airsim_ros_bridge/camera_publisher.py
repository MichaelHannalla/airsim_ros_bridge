#! /usr/bin/env python

import cv2
import rospy
import airsim
import matplotlib.pyplot as plt 
import numpy as np

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from sensor_msgs.msg import Image, CameraInfo
import ros_numpy

client = airsim.CarClient()
client.confirmConnection()

#Initializing ros node and publishers. 
rospy.init_node('camera_node')

# Publishers for images
img_pub = rospy.Publisher('airsim/mono/image_raw', Image, queue_size=1)
img_depth_pub = rospy.Publisher('airsim/mono/depth', Image, queue_size=1) 
img_r_pub = rospy.Publisher('airsim/stereo/left/image_raw', Image, queue_size=1) 
img_l_pub = rospy.Publisher('airsim/stereo/right/image_raw', Image, queue_size=1) 

# Publishers for intrinsics
#img_info_pub = rospy.Publisher('airsim/mono/camera_info', CameraInfo, queue_size=1)
#img_l_info_pub = rospy.Publisher('airsim/stereo/left/camera_info', CameraInfo, queue_size=1)
#img_r_info_pub = rospy.Publisher('airsim/stereo/right/camera_info', CameraInfo, queue_size=1)

ros_img = Image()
ros_img_depth = Image()
ros_img_r = Image()
ros_img_l = Image()

ros_img.header.frame_id = "camera_link"
ros_img_depth.header.frame_id = "camera_link"
ros_img_r.header.frame_id = "right_stereo_link"
ros_img_l.header.frame_id = "left_stereo_link"
r = rospy.Rate(30)

while not rospy.is_shutdown():
    responses = client.simGetImages([ airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False), 
        airsim.ImageRequest("front_center", airsim.ImageType.DepthPlanner, True, False),
        airsim.ImageRequest("front_left", airsim.ImageType.Scene, False, False),
        airsim.ImageRequest("front_right", airsim.ImageType.Scene, False, False)]) 

    # Preparing the front mono image
    img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8) #get numpy array
    img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)  #reshape array to 3 channel image array H X W X 3
    img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    
    # Preparing the depth map
    depth_map = responses[1].image_data_float
    depth_map = np.reshape(depth_map, (responses[1].height, responses[1].width))

    # Preparing the left stereo image
    img1d_left = np.frombuffer(responses[2].image_data_uint8, dtype=np.uint8) #get numpy array
    img_rgb_left = img1d_left.reshape(responses[2].height, responses[0].width, 3)  #reshape array to 3 channel image array H X W X 3

    # Preparing the left stereo image
    img1d_right = np.frombuffer(responses[3].image_data_uint8, dtype=np.uint8) #get numpy array
    img_rgb_right = img1d_right.reshape(responses[3].height, responses[0].width, 3)  #reshape array to 3 channel image array H X W X 3

    # Preparing the front mono rgb and depth image ROS messages
    ros_img = ros_numpy.msgify(Image, img_bgr, encoding="rgb8")
    ros_img.header.frame_id = "camera_link"
    ros_img.header.stamp = rospy.Time.now()

    # temporary method for getting tranformation between both left and right cams
    #cam_left_info  = client.simGetCameraInfo("front_left")
    #cam_right_info = client.simGetCameraInfo("front_right")
    #print(cam_left_info.pose.position - cam_right_info.pose.position)

    # Preparing the left stereo image ROS message
    ros_img_l = ros_numpy.msgify(Image, img_rgb_left, encoding="bgr8")
    ros_img_l.header.frame_id = "camera_link"
    ros_img_l.header.stamp = ros_img.header.stamp

    # Preparing the right stereo image ROS message
    ros_img_r = ros_numpy.msgify(Image, img_rgb_right, encoding="bgr8")
    ros_img_r.header.frame_id = "camera_link"
    ros_img_r.header.stamp = ros_img.header.stamp

    # Publish the ROS messages
    img_pub.publish(ros_img)
    img_l_pub.publish(ros_img_l)
    img_r_pub.publish(ros_img_r)

    r.sleep()
