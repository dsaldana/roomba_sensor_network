#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from roomba_comm.msg import SensedValue


from math import *
import random

# Gazebo
import gazebo_msgs.srv

import copy

from roomba_sensor.sensor import Sensor
#import roomba_sensor.hello

goal = Point32()
goal.x=-1
goal.y=-1

def cut_angle(angle):	
	if angle > pi:
		angle -= 2*pi
	if -angle < -pi:
		angle += 2*pi
	return angle


def goal_callback(point):
	global goal
	goal = point
	print "new target ", goal

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
	image_sub = rospy.Subscriber(topicName, Point32, goal_callback)

	print "wait for service"
	rospy.wait_for_service('/gazebo/get_model_state')
	getPosition = rospy.ServiceProxy('/gazebo/get_model_state', gazebo_msgs.srv.GetModelState)

	# Robot position
	robotX = float("inf")
	robotY = float("inf")

	######## Control Loop ###########
	print "Start!"
	while not rospy.is_shutdown():
		try:			
			resp = getPosition(robotName,"world")
			robotX = resp.pose.position.x
			robotY = resp.pose.position.y
			# the reference for the angle is the y axes.
			robotT = resp.pose.orientation.z 
			print "position ", (resp.pose.orientation)
		except rospy.ServiceException, e:
			print "Service call to gazebo failed: %s" %e

		[sX, sY, sT] = [robotX, robotY, robotT]

		# Orientation
		x = goal.x - sX 
		y = goal.y - sY
		# the reference for the angle is the y axes.
		t = atan(y/x) 
		if  x<0:
			print "...."
			t += pi
		t = cut_angle(t)

		#print [x,y], " to "

		controlT = t - robotT
		controlT = cut_angle(controlT)
	
		#if controlT < 0:
		#	controlT += 2*pi

		#print "angle: ", degrees(t), " diff: ", degrees(controlT)
		
		vel = Twist()
		vel.linear.x = 0
		vel.angular.z = (controlT) / 3
		#velPub.publish(vel)



		rospy.sleep(0.50)


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
