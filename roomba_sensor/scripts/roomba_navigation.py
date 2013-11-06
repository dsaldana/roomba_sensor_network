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
from roomba_sensor.roomba import RoombaGazebo
from roomba_sensor.util import cut_angle

goal = Point32()
goal.x = -1
goal.y = -1

# state
traking = False

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
	# Node roomba navigation
	rospy.init_node('roomba_navigation')

	#a = Sensor()
	#print a.p
	#roomba_sensor.hello.say('my friend!')
		
	######### Initialization ##################
	# Robot's name is an argument
	global robotName
	robotName = rospy.get_param('~robot_name', 'Robot1')

	# Create the Publisher to control the robot.
	topicName = "/" + robotName + "/commands/velocity"
	velPub = rospy.Publisher(topicName, Twist)
		
	topicName = "/" + robotName + "/goal"
	rospy.Subscriber(topicName, Point32, goal_callback)

	# Tracking callback
	topicName = "/" + robotName + "/tracking"
	rospy.Subscriber(topicName, Float32, tracking_callback)

	# Object to get information from Gazebo
	robot = RoombaGazebo(robotName)


	######## Control Loop ###########
	print "Start!"
	while not rospy.is_shutdown():
		
		if(traking):
			### Tracking ###
			lin_vel = 0.3
			max_angle = pi / 3

			vel = Twist()
			vel.linear.x = lin_vel
			vel.angular.z = sensedValue.data * max_angle
			velPub.publish(vel)

		else:
			### Navigate ###
			[sX, sY, sT] = robot.getPosition()

			# Orientation
			x = goal.x - sX 
			y = goal.y - sY
			# the reference for the angle is the x axes.
			t = atan(y / x) 
			if  x < 0:
				t += pi
			t = cut_angle(t)

			controlT = t - sT
			controlT = cut_angle(controlT)		
			
			# Euclidean distance
			d = sqrt(x*x + y*y)

			print  "distance=",d," teta: ", degrees(controlT)

			vel = Twist()
			vel.linear.x = d / 5
			vel.angular.z = (controlT) / 1
			velPub.publish(vel)

		rospy.sleep(0.20)


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
