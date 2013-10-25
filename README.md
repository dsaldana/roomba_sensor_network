roomba_sensor_network
=====================

A robotic sensor network based on roomba robots.



Required ROS Packages
- ros-hydro-hector-sensors-description
- ros-hydro-turtlebot-simulator
- ros-hydro-desktop-full*


Run
$ roslaunch turtlebot_ds turtlebot_multi_world.launch

To see the particles for robot 1
$ rosrun roomba_sensor particle_drawer.py

To see the robot cam
$ rosrun image_view image_view image:=/Robot1/front_cam/camera/image



