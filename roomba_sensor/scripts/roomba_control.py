#!/usr/bin/env python

# Numpy
import math
import rospy
# Math
from roomba_sensor.control.gradientdescent import GradientDescent

from roomba_sensor.comm.communicator import Communicator
from roomba_sensor.geometric.AnomalyManager import AnomalyManager
from roomba_sensor.localization import RoombaLocalization
from roomba_sensor.control.particlefilter import ParticleFilter
from roomba_sensor.geometric.vector import points_to_vector

from roomba_sensor.sensor.sensor_camera import Camera


GRAPHIC_DEBUG = True


def run():
    # ######## Initialization ##################
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
    # # Robot Localization
    robot = RoombaLocalization(robot_name)
    # Path line in anomaly detection for each robot.
    am = AnomalyManager(robot_name)
    # Initialize Particles
    pf = ParticleFilter()

    if GRAPHIC_DEBUG:
        from roomba_sensor.viewer.robotdrawer import RobotDrawer

        display = RobotDrawer()

    # #
    # ####### Control Loop ###################
    while not rospy.is_shutdown():
        # Get robot position from gazebo
        robot_position = robot.get_position()

        [robot_x, robot_y, robot_t] = robot_position

        # Read all messages from other robots.
        orobots, sensed_points, anomaly_polygons = communicator.read_inbox()

        am.add_sensed_points(sensed_points)
        am.data_polygons = anomaly_polygons

        # print am.anomaly_full, am.is_polygon_identified
        # Send the info to other robots.
        if am.is_polygon_identified:
            # Includes time of detection and closed anomaly
            communicator.send_sensed_value(camera.sensed_value, robot.get_sensor_position(), robot_position,
                                           polygon=am.polyline, closed_anomaly=am.anomaly_full, time_of_detection=0)
        else:
            communicator.send_sensed_value(camera.sensed_value, robot.get_sensor_position(), robot_position)

        # Particle filter: move the particles for simulating the anomaly's dynamics
        pf.move_particles()

        # FIXME quiza esto deberia estar en otro lugar
        am.fix_polygon()

        # Particle filter: update based on sensor value and detected anomalies.
        pf.update_particles(sensed_points.values(), anomaly_polygons)

        # Particle filter: Re-sampling.
        pf.resample()

        # Publish particles
        communicator.publish_particles(pf.particles, robot_position, orobots, am.polyline)

        # print anomaly_polygons
        ######## Exploring #################
        if not am.sensed_anomaly:
            # Total force
            total_force = planner.compute_forces(pf, robot_x, robot_y, communicator.robot_msgs.values())
            # Compute goal
            goal = robot_x + total_force[0], robot_y + total_force[1]
            # Publish goal to navigate
            communicator.publish_goal(goal)

        else:
            ####### Tracking ##########
            # modify the polygon with sensed data and other robot's data
            am.modify_polygon()

            # close the polygon if necessary
            am.evaluate_anomaly_full()

            #FIXME quiza esto deberia estar en otro lugar
            am.fix_polygon()

            # Compute Proportional control for steering
            # control_p = (camera.sensed_left - 1) + camera.sensed_right

            # Convert a value from [0, 1] to a value in the interval [-1, 1].
            control_p = camera.sensed_value * 2 - 1

            # counter robot force.
            crf = 0

            # Observe the other robots to avoid coalitions controlling the
            # linear velocity.
            for r in communicator.robot_msgs.values():
                if r.robot_id == robot_name:
                    continue

                # Vector to the other robot
                d, theta = points_to_vector([robot_x, robot_y], [r.rx, r.ry])
                # Robot force.
                rf = 1 / (d ** 2)
                # is this robot considerable?
                # if the other robot is in front of it (angle view is 120 degrees)
                if abs(theta - robot_t) < (math.pi / 3):
                    if rf > crf:
                        crf = rf

            communicator.publish_track(control_p, crf)

        if GRAPHIC_DEBUG:
            display.clear()
            display.draw_robot(robot_position[:2], robot_position[2])

            display.draw_path(am.polyline)
            # for r, p in anomaly_polygons.items():
            #     # poly = [(pol.x, pol.y) for pol in p]
            #     print p[1], p[2]


            display.draw()

        rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
