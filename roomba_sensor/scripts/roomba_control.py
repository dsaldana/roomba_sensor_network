#!/usr/bin/env python

# Numpy

import rospy



# Math
from math import *
from roomba_sensor.gradient_descent import GradientDescent

from roomba_sensor.communicator import Communicator
from roomba_sensor.geometric.AnomalyManager import AnomalyManager
from roomba_sensor.roomba import RoombaLocalization
from roomba_sensor.particle_filter import ParticleFilter
from roomba_sensor.geometric.vector import points_to_vector

from roomba_sensor.sensor_camera import Camera


def run():
    ######### Initialization ##################
    # Node roomba_control
    rospy.init_node('roomba_control')

    # Robot's name is an argument
    robot_name = rospy.get_param('~robot_name', 'Robot1')
    rospy.loginfo("Loading robot control for " + robot_name)

    # Communication
    communicator = Communicator(robot_name)
    # Camera
    camera = Camera(robot_name)
    # Gradient descent
    planner = GradientDescent(robot_name)
    ## Robot Localization
    robot = RoombaLocalization(robot_name)
    # Path line in anomaly detection for each robot.
    am = AnomalyManager(robot_name)


    # Initialize Particles
    pf = ParticleFilter()

    # Explore flag. False for tracking
    explore = True

    # Time for tracking without sensing anomaly.
    max_tracking_time = 5
    # last time that an anomaly was detected
    last_time_anomaly = 0



    ######## Control Loop ###################
    while not rospy.is_shutdown():
        # Get robot position from gazebo
        robot_position = robot.get_position()
        [robot_x, robot_y, robot_t] = robot_position

        # Read all messages from other robots.
        orobots, sampled_points, polygons = communicator.read_inbox(am)

        # Send the info to other robots.
        communicator.send_sensed_value(camera.sensed_value, robot.get_sensor_position(), robot_position)


        # Process anomaly lines
        am.process()

        # Particle filter: move the particles for simulating the anomaly's dynamics
        pf.move_particles()
        # Particle filter: update based on sensor value.
        #TODO add polygons
        pf.update_particles(sampled_points)

        # Particle filter: Re-sampling.
        pf.resample()

        # Publish particles
        communicator.publish_particles(pf.particles, robot_position, orobots, am.get_anomaly_points())

        #
        #### Evaluate Sensed value
        # The flag for exploring change only if  the robot does not
        # sense an anomaly in a n seconds (n = max_tracking_time).
        if camera.sensed_value > 0:
            explore = False
            last_time_anomaly = rospy.get_rostime().secs
        else:
            if rospy.get_rostime().secs - last_time_anomaly > max_tracking_time:
                explore = True

        ######## Exploring #################
        if explore:

            # Total force
            total_force = planner.compute_forces(pf, robot_x, robot_y, communicator.robot_msgs.values())
            # Compute goal
            goal = robot_x + total_force[0], robot_y + total_force[1]
            # Publish goal to navigate
            communicator.publish_goal(goal)

        else:
            ####### Tracking ##########
            control_p = (camera.sensed_left - 1) + camera.sensed_right

            # counter robot force.
            crf = 0

            ### TODO Observe the other robots
            for r in communicator.robot_msgs.values():
                if r.robot_id == robot_name:
                    continue

                # Vector to the other robot
                d, theta = points_to_vector([robot_x, robot_y], [r.rx, r.ry])
                # Robot force.
                rf = 1 / (d ** 2)
                # is this robot considerable?
                # if the other robot is in front of it (angle view is 120 degrees)
                if abs(theta - robot_t) < (pi / 3):
                    if rf > crf:
                        crf = rf

            communicator.publish_track(control_p, crf)

        rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
