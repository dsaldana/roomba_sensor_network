#!/usr/bin/env python

# Numpy
import numpy as np

import rospy


# Clustering
from scipy.cluster.vq import kmeans2
# Math
from math import *

from roomba_sensor.communicator import Communicator
from roomba_sensor.geometric.AnomalyManager import AnomalyManager
from roomba_sensor.roomba import RoombaLocalization
from roomba_sensor.particle_filter import ParticleFilter
from roomba_sensor.geometric.vector import points_to_vector, vector_components


from roomba_sensor.sensor_camera import Camera




def run():
    ######### Initialization ##################
    # Node roomba_control
    rospy.init_node('roomba_control')

    # Robot's name is an argument
    robot_name = rospy.get_param('~robot_name', 'Robot1')

    # Constant in columbs law. Force by centroid.
    f_centroid = rospy.get_param('/f_centroid', 1.0)
    f_robots = rospy.get_param('/f_robots', 2.0)

    rospy.loginfo("Loading robot control for " + robot_name)

    # Communication
    communicator = Communicator(robot_name)

    # Camera
    camera = Camera(robot_name)

    # Initialize Particles
    pf = ParticleFilter()

    ## Robot Localization
    robot = RoombaLocalization(robot_name)

    # Explore flag. False for tracking
    explore = True

    # Time for tracking without sensing anomaly.
    max_tracking_time = 5
    # last time that an anomaly was detected
    last_time_anomaly = 0

    # Path line in anomaly detection for each robot.
    am = AnomalyManager(robot_name)

    cents = None

    k_skip = -1

    ######## Control Loop ###################
    print "Start!"
    while not rospy.is_shutdown():
        # Get robot position from gazebo
        robot_position = robot.get_position()
        [robot_x, robot_y, robot_t] = robot_position

        # Send the info to other robots.
        communicator.send_sensed_value(camera.sensed_value, robot.get_sensor_position(), robot_position)

        # Particle filter: move the particles for simulating the anomaly's dynamics
        pf.move_particles()

        # Read all messages from other robots.
        orobots, samples = communicator.read_inbox(am)

        # Process anomaly lines
        am.process()

        # Particle filter: update based on sensor value.
        pf.update_particles(samples)

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
            k_groups = 9
            #
            x, y = [], []
            for p in pf.particles:
                x.append(p.x)
                y.append(p.y)

            # Categorize in k groups and compute
            # new centroids
            np.random.seed(1)

            if k_skip < 0:
                cents, idx = kmeans2(np.array(zip(x, y)), k_groups)
                k_skip = 5
            else:
                k_skip -= 1

            # Total force
            total_force = [0, 0]
            # Forces by the clusters
            fc = []
            for i in range(len(cents)):
                c = cents[i]

                # points in centroid
                n_pts = sum(idx == i)

                # Vector to the centroid
                d, theta = points_to_vector([robot_x, robot_y], c)

                # Force. Coulomb law. Charge c=n_pts
                fm = f_centroid * n_pts / (d ** 2)

                # components
                u, v = vector_components(fm, theta)

                total_force[0] += u
                total_force[1] += v

                fc.append([u, v])

            ## Forces by other robots
            try:
                for r in communicator.robot_msgs.values():
                    if r.robot_id == robot_name:
                        continue

                    # Vector to the other robot
                    d, theta = points_to_vector([robot_x, robot_y], [r.rx, r.ry])

                    # Foce, Coulombs law. Charge c=n_pts
                    k = f_robots * (len(pf.particles) / len(cents))
                    fm = k / (d ** 2)

                    # Components
                    u, v = vector_components(fm, theta)

                    # Positive or robot Force is in opposite direction.
                    total_force[0] -= u
                    total_force[1] -= v

            except Exception, e:
                rospy.logerr("Error integrating the data from other robots. " + str(e))

            cte = 10.0 / (len(pf.particles))
            total_force = cte * total_force[0], cte * total_force[1]
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
                # print [r.rx, r.ry]  # , "d=",d," theta=", theta
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
