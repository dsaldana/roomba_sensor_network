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

# P Control factors
p_angular = rospy.get_param('/p_control_angular', 1.0)
p_linear = rospy.get_param('/p_control_linear', 0.5)


# Sensed value is between 0 e 1
def tracking_callback(sensedData):
    """

    :param sensedData:
    """
    global tracking
    global sensedValue
    global crf

    # sensed data
    sensedValue = sensedData.x
    crf = sensedData.y
    traking = True
    print "Tracking=", sensedValue, " force=", crf


def goal_callback(point):
    """

    :param point:
    """
    global goal
    global tracking
    global spiral_counter
    traking = False
    goal = point

    if not point.z == -1:
        spiral_counter = 10
    #print "new goal ", [goal.x, goal.y,  goal.z]


def run():
    """

    """
    global tracking
    global spiral_counter
    # Node roomba navigation
    rospy.init_node('roomba_navigation')

    ######### Initialization ##################
    # Robot's name is an argument
    global robot_name
    robot_name = rospy.get_param('~robot_name', 'Robot1')
    simulated_robots = rospy.get_param('/simulated_robots', False)


    # Create the Publisher to control the robot.
    topicName = "/" + robot_name + "/commands/velocity"
    velPub = rospy.Publisher(topicName, Twist)

    topicName = "/" + robot_name + "/goal"
    rospy.Subscriber(topicName, Point32, goal_callback, queue_size=1)

    # Tracking callback
    topicName = "/" + robot_name + "/tracking"
    rospy.Subscriber(topicName, Point32, tracking_callback, queue_size=1)

    ## Object to get information from Gazebo
    robot = RoombaLocalization(robot_name)

    ######## Control Loop ###########
    print "Start!"
    old_val = -1
    spiral_counter = 10

    while not rospy.is_shutdown():
        rospy.sleep(0.20)

        #if (traking is None):
        #	continue
        #TODO implement tracking in another module.
        if (traking):
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

            P = pi / 4
            D = pi / 2

            vel = Twist()
            vel.linear.x = 0.3 * lin_vel
            vel.angular.z = 0.5 * -(sensedValue * P + (sensedValue - old_val) * D)

            velPub.publish(vel)

            old_val = sensedValue

        else:
            ###### Navigation #####
            [sX, sY, sT] = robot.get_position()

            ###### Spiral exploration #####
            if goal.z == -1:

                # Angular velocity
                a = pi / 3
                b = -0.01
                av = a * exp(b * spiral_counter)
                # Linear velocity
                #lv = exp(0.01 * spiral_counter)
                lv = 0.3 * (1 - exp(-spiral_counter))

                spiral_counter += 1
                print spiral_counter, " lv=", lv, " av=", av

                maxLV = 0.5
                if lv > maxLV:
                    lv = maxLV


                # Send control message
                vel = Twist()
                vel.linear.x = lv
                vel.angular.z = av
                velPub.publish(vel)


            else:
                ##### Navigate to a point. ###########
                print "Navigating ", [sX, sY, sT]
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
                linear_r = [0.02, 0.5]

                if vel.linear.x > linear_r[1]:
                    vel.linear.x = linear_r[1]

                angular_r = [-pi, pi]
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
