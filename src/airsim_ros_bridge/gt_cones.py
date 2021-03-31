#! /usr/bin/env python

import numpy as np
import rospy
import matplotlib.pyplot as plt
import message_filters
from asurt_msgs.msg import LandmarkArray, Landmark
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import airsim

path = rospy.get_param("utils_path")
gt_blue_cones = np.genfromtxt(path + 'blue_cone_locations.txt', usecols=(0,1))
gt_yellow_cones = np.genfromtxt(path + 'yellow_cone_locations.txt', usecols=(0,1))
gt_large_cones = np.genfromtxt(path + 'orange_cone_locations.txt', usecols=(0,1))
car_init = np.genfromtxt(path + 'car_start_location.txt', usecols=(0,1)).reshape(-1,2)

gt_blue_cones   -= car_init       # Relative to vehicle start location (0, 0)
gt_yellow_cones -= car_init
gt_large_cones  -= car_init
gt_blue_cones   /= 100
gt_yellow_cones /= 100            # Conversion to SI units
gt_large_cones /= 100            # Conversion to SI units
gt_blue_cones[:,1]   *= -1
gt_yellow_cones[:,1] *= -1            # Conversion to SI units
gt_large_cones[:,1] *= -1            # Conversion to SI units

client = airsim.CarClient()


def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
    
def get_trans():
    car_state = client.getCarState()
    x = car_state.kinematics_estimated.position.x_val
    y = -1 * car_state.kinematics_estimated.position.y_val
    wo = car_state.kinematics_estimated.orientation.w_val
    xo = car_state.kinematics_estimated.orientation.x_val
    yo = car_state.kinematics_estimated.orientation.y_val
    zo = -1 * car_state.kinematics_estimated.orientation.z_val
    quat = (xo, yo, zo, wo)
    _,_, th = euler_from_quaternion(quat)
    theta = pi_2_pi(th)
    rot = np.mat([
      [ np.cos(theta), -np.sin(theta)],
      [ np.sin(theta),  np.cos(theta)]])
    trans = np.mat([[x], [y]])
    return rot.T, -rot.T @ trans

def main():

    rospy.init_node('airsim_gt_cones_detector_node')
    cones_pub = rospy.Publisher('airsim/Cones', LandmarkArray, queue_size=1) 
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
      rot, trans = get_trans()
      blue_in_vehicle = np.asarray(rot @ gt_blue_cones.T + trans).T
      yellow_in_vehicle = np.asarray(rot @ gt_yellow_cones.T + trans).T
      large_in_vehicle = np.asarray(rot @ gt_large_cones.T + trans).T

      blue_in_range = blue_in_vehicle[blue_in_vehicle[:,0] >= 1.58]
      yellow_in_range = yellow_in_vehicle[yellow_in_vehicle[:,0] >= 1.58]
      large_in_range = large_in_vehicle[large_in_vehicle[:,0] >= 1.58]

      dist_blue = np.sqrt((blue_in_range[:,0]**2) + (blue_in_range[:,1]**2))
      dist_yellow = np.sqrt((yellow_in_range[:,0]**2) + (yellow_in_range[:,1]**2))
      dist_orange = np.sqrt((large_in_range[:,0]**2) + (large_in_range[:,1]**2))

      blue_in_range = blue_in_range[dist_blue < 15]
      yellow_in_range = yellow_in_range[dist_yellow < 15]
      large_in_range = large_in_range[dist_orange < 15]

      lms = LandmarkArray()
      
      for blue_cone in blue_in_range:
        lm = Landmark()
        lm.position.x = blue_cone[0] - 1.58
        lm.position.y = blue_cone[1]
        lm.type = 0
        lms.landmarks.append(lm)

      for yellow_cone in yellow_in_range:
        lm = Landmark()
        lm.position.x = yellow_cone[0] - 1.58
        lm.position.y = yellow_cone[1]
        lm.type = 1
        lms.landmarks.append(lm)
      
      for large_cone in large_in_range:
        lm = Landmark()
        lm.position.x = large_cone[0] - 1.58
        lm.position.y = large_cone[1]
        lm.type = 3
        lms.landmarks.append(lm)

      if len(lms.landmarks) > 0:
        lms.header.stamp = rospy.Time.now()
        lms.header.frame_id = "base_link"
        #print("publishing")
        cones_pub.publish(lms)

      if False:
        plt.cla()
        ax = plt.gca()
        for landmark in lms.landmarks:
          if landmark.type == 0:   
            plt.plot(landmark.position.x, landmark.position.y, "xb")
              
          elif landmark.type == 1:
            plt.plot(landmark.position.x, landmark.position.y, "xy")
                  
          elif landmark.type == 3:
            plt.plot(landmark.position.x, landmark.position.y, "xr")
        
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)
      
      r.sleep()

if __name__ == '__main__':
    main()

