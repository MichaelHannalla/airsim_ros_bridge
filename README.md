# airsim_ros_bridge
This is a Python ROS Package for publishing [AirSim simulator](https://github.com/FSTDriverless/AirSim) (based on Unreal Engine) on ROS as topics, and implementing nodes for different autonomous software stack algorithms.

This package was initially developed for [ASU Racing Team](https://www.facebook.com/ASU.Racing.Team/), Formula Student AI Team for participating in FS-AI 2020 Online Simulation Development Event.
This package is tested for ROS 1.0 Kinetic Kame using Python 2.7

## Published topics:
* GPS: airsim/gps/fix
* IMU: airsim/imu
* Lidar: airsim/VelodynePoints
* Odom: odom
* Right cones array: airsim/RightCones
* Left cones array: airsim/LeftCones

## Python scripts:
* cones_detector.py: Publishes two PoseArray messages for the right and left cones.
* delaunay_planner.py: Publishes PoseArray message for the waypoints.
* gps_publisher.py: Publishes NavSatFix message for GPS.
* odometry_publisher.py: Publishes Odometry messgae estimated from kinematics.
* pointcloud_publisher.py: Publishes Imu and PointCloud2 messages for the IMU and lidar.
* tf_to_odom_publisher.py: Publishes Odometry message by listening to the map to odom transform.

