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

from sensor_msgs.msg import Image
import ros_numpy

client = airsim.CarClient()
client.confirmConnection()

#Initializing ros node and publishers. 
rospy.init_node('camera_node')
img_pub = rospy.Publisher('airsim/image_raw', Image, queue_size=10) 
ros_img = Image()
ros_img.header.frame_id = "camera_link"
r = rospy.Rate(50)

while not rospy.is_shutdown():
    responses = client.simGetImages([ airsim.ImageRequest("0", airsim.ImageType.Scene, False, False), 
        airsim.ImageRequest("front_center", airsim.ImageType.DepthPlanner, True, False)]) 

    img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8) #get numpy array
    img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)  #reshape array to 3 channel image array H X W X 3
    img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    depth_map = responses[1].image_data_float
    depth_map = np.reshape(depth_map, (responses[1].height, responses[1].width))

    ros_img = ros_numpy.msgify(Image, img_bgr, encoding="rgb8")
    ros_img.header.stamp = rospy.Time.now()
    img_pub.publish(ros_img)
    r.sleep()

    
