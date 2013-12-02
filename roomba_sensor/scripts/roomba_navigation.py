#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from roomba_comm.msg import SensedValue


from math import *
import random

import copy

from roomba_sensor.sensor import Sensor
from roomba_sensor.roomba import RoombaLocalization
from roomba_sensor.util import cut_angle

goal = Point32()
goal.x = -1
goal.y = -1

# state
traking = None

# Sensed value is between 0 e 1
def tracking_callback(sensedData):
	global traking
	global sensedValue

	sensedValue = sensedData
	traking = True
	print "Tracking ", sensedValue

def goal_callback(point):
	global goal
	global traking
	traking = False
	goal = point
	print "new goal ", [goal.x, goal.y]

def run():
	global traking
	# Node roomba navigation
	rospy.init_node('roomba_navigation')
	
	######### Initialization ##################
	# Robot's name is an argument
	global robot_name
	robot_name = rospy.get_param('~robot_name', 'Robot1')
	simulated_robots = rospy.get_param('simulated_robots', True)


	# Create the Publisher to control the robot.
	topicName = "/" + robot_name + "/commands/velocity"
	velPub = rospy.Publisher(topicName, Twist)
		
	topicName = "/" + robot_name + "/goal"
	rospy.Subscriber(topicName, Point32, goal_callback)

	# Tracking callback
	topicName = "/" + robot_name + "/tracking"
	rospy.Subscriber(topicName, Float32, tracking_callback)

	## Object to get information from Gazebo
	robot = RoombaLocalization(robot_name)	

	######## Control Loop ###########
	print "Start!"
	old_val = -1
	while not rospy.is_shutdown():
		rospy.sleep(0.20)

		if (traking is None):
			continue
		#TODO implement tracking in another module.
		if(traking):
			###### Tracking ######
			lin_vel = 0.2
			P = pi / 4
			D = pi / 2

			vel = Twist()
			vel.linear.x = 0.3 * lin_vel
			vel.angular.z = 0.6 * -(sensedValue.data * P + (sensedValue.data - old_val) * D)
			
			velPub.publish(vel)

			old_val = sensedValue.data

		else:
			###### Navigation ######
			[sX, sY, sT] = robot.get_position()

			# Orientation
			x = goal.x - sX 
			y = goal.y - sY

			if(x == 0):
				rospy.sleep(0.20)
				continue

			# The reference for the angle is the x axes.
			# TODO cambiar por atan2
			t = atan(y / x) 
			if  x < 0:
				t += pi
			t = cut_angle(t)

			controlT = t - sT
			controlT = cut_angle(controlT)		
			
			# Euclidean distance
			d = sqrt(x*x + y*y)

			print  "distance=", d, " teta: ", degrees(controlT)

			vel = Twist()
			vel.linear.x = d / 5
			vel.angular.z = (controlT) / 1

			# Max velocities
			max_linear_speed = 3
			max_angular_speed = 100
			#if vel.linear.x > max_linear_speed:
			#	vel.linear.x = max_linear_speed
			if vel.angular.z > max_angular_speed:
				vel.angular.z = max_angular_speed
			
			velPub.publish(vel)

		


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
