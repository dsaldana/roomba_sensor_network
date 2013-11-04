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

#from roomba_sensor import Sensor
import roomba_sensor.hello

goal = Point32()

def goal_callback(point):
	global goal
	goal = point
	print "new target ", goal

def run():
	#a = Sensor()
	roomba_sensor.hello.say('my friend!')
	return	
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
			robotT = resp.pose.orientation.z
			#print "position ", degrees(resp.pose.orientation.z)
		except rospy.ServiceException, e:
			print "Service call to gazebo failed: %s" %e


		vel = Twist()
		vel.linear.x = 0
		vel.angular.z = (camY-theta)/10
		#velPub.publish(vel)

		rospy.sleep(0.50)


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
