roomba_sensor_network
=====================

A robotic sensor network based on roomba robots.

* Required ROS Packages
    - ros-indigo-hector-sensors-description
    - ros-indigo-turtlebot-simulator
    - ros-indigo-kobuki-gazebo


* Python packages:
    - sudo apt-get install python-shapely python-pygame python-scipy

* Copy the folder 1fogo from doc/gazebomodels/ to ~/.gazebo/models/

* Run for simulated robots
    ```bash
    # Run Gazebo Simulator
    $ roslaunch turtlebot_ds roomba_simulation.launch


    # To see the particles for robot 1
    $ rosrun roomba_sensor particle_drawer2.py

    # To see the robot cam
    $ rosrun image_view image_view image:=/Robot1/front_cam/camera/image
    ```


sudo apt-get install ros-indigo-turtlebot ros-indigo-cmake-modules ros-indigo-create-driver ros-indigo-usb-cam             








