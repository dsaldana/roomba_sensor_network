roomba_sensor_network
=====================

A robotic sensor network based on roomba robots.

* Required ROS Packages
    - ros-hydro-hector-sensors-description (https://github.com/tu-darmstadt-ros-pkg/hector_models/tree/indigo-devel)
    - ros-hydro-turtlebot-simulator
    - ros-hydro-desktop-full*

* Python packages:
    - shapely
    - pygame
    - scipy

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



