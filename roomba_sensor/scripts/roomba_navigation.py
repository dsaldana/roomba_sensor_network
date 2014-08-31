#!/usr/bin/env python
from math import *

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32

from roomba_sensor.localization import RoombaLocalization
from roomba_sensor.util.angle import cut_angle


goal = Point32()
goal.x = 0.0000001
goal.y = 0.0000001

# state
tracking = False
new_tracking_msg = False
# P Control constants for navigation
p_linear = rospy.get_param('/p_control_linear', 0.5)
p_angular = rospy.get_param('/p_control_angular', 1.0)


# PD Control for tracking
# P = pi / 4
# D = pi / 1
P_TRACKING = 0.70
D_TRACKING = 0.5

# Sensed value is between 0 e 1
def tracking_callback(sensedData):
    global tracking
    global sensedValue
    global crf
    global new_tracking_msg

    # sensed data
    sensedValue = sensedData.x
    crf = sensedData.y
    tracking = True
    new_tracking_msg = True


def goal_callback(point):
    global goal
    global tracking
    tracking = False
    goal = point


def run():
    global tracking
    global new_tracking_msg

    # Node roomba navigation
    rospy.init_node('roomba_navigation')

    # ######## Initialization ##################
    # Robot's name is an argument
    global robot_name
    robot_name = rospy.get_param('~robot_name', 'Robot1')
    simulated_robots = rospy.get_param('/simulated_robots', False)


    # Create the Publisher to control the robot.
    topicName = "/" + robot_name + "/commands/velocity"
    velPub = rospy.Publisher(topicName, Twist, queue_size=1)

    topicName = "/" + robot_name + "/goal"
    rospy.Subscriber(topicName, Point32, goal_callback, queue_size=1)

    # Tracking callback
    topicName = "/" + robot_name + "/tracking"
    rospy.Subscriber(topicName, Point32, tracking_callback, queue_size=1)

    # # Object to get information from Gazebo
    robot = RoombaLocalization(robot_name)

    # ####### Control Loop ###########
    print "Start!"
    old_val = 0
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

        # if (traking is None):
        # continue
        # TODO implement tracking in another module.
        if tracking:
            if new_tracking_msg:
                new_tracking_msg = False

                ###### Tracking ######
                # Max speed for tracking.
                lin_vel = 0.4
                # if there is a robot to crash.
                if crf > 0:
                    # input force crf: i = [0 , 1.2]
                    # output o = [0.4, 0]
                    # mappint i to o: x = (2 - i) * 0.4
                    cm = 4.5
                    lin_vel *= (cm - crf) / cm

                    if lin_vel < 0:
                        lin_vel = 0

                vel = Twist()
                vel.linear.x = 0.3 * lin_vel
                vel.angular.z = -(sensedValue * P_TRACKING + (sensedValue - old_val) * D_TRACKING)

                # max_angle = pi / 3
                # if vel.angular.z > max_angle:
                #     vel.angular.z = max_angle
                # elif vel.angular.z < - max_angle:
                #     vel.angular.z = - max_angle

                # print lin_vel,degrees(vel.angular.z)
                print degrees(vel.angular.z), sensedValue, old_val
                velPub.publish(vel)

                old_val = sensedValue

        else:
            old_val = 0
            ###### Navigation #####
            [sX, sY, sT] = robot.get_position()
            print "Navigating", [sX, sY, sT]

            # Orientation
            x = goal.x - sX
            y = goal.y - sY

            if (x == 0):
                rospy.sleep(0.20)
                continue

            # The reference for the angle is the x axes.
            t = atan(y / x)
            if x < 0:
                t += pi
            t = cut_angle(t)

            controlT = t - sT
            controlT = cut_angle(controlT)

            # Euclidean distance
            d = sqrt(x * x + y * y)

            print  "distance=", d, " teta: ", degrees(controlT)

            vel = Twist()
            # for rial robot: vel.linear.x = 0.5 * d
            ### P Control ###

            vel.linear.x = p_linear * d
            vel.angular.z = p_angular * controlT

            # For real robots: the relative position affect the axes.
            if not simulated_robots:
                vel.angular.z *= -1

            # velocity range
            linear_r = [0.02, 0.2]
            angular_r = [-pi, pi]

            if vel.linear.x > linear_r[1]:
                vel.linear.x = linear_r[1]

            if vel.angular.z > angular_r[1]:
                vel.angular.z = angular_r[1]
            elif vel.angular.z < - angular_r[1]:
                vel.angular.z = - angular_r[1]
            #if vel.angular.z < angular_r[0]:
            #	vel.angular.z = 0

            velPub.publish(vel)


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
