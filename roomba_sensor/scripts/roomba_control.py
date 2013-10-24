#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

import random

# For plotting
import numpy as np
import matplotlib.pyplot as plt
# Gazebo
#import roslib; roslib.load_manifest('gazebo')
#from gazebo.srv import *
import gazebo_msgs.srv


def run():
	######### Initialization ##################
	# Node roombaControl
	rospy.init_node('roomba_control')

	# Robot's name is an argument
	robotName = rospy.get_param('~robot_name', 'Robot1')
	print robotName

	# Create the Publisher to control the robot.
	topicName = "/" + robotName + "/commands/velocity"
	velPub = rospy.Publisher(topicName, Twist)

	# Create a publisher for the particles
	partPub = rospy.Publisher(robotName+"/particles", Polygon)


	#TODO this code must be out of this file
	########## Initialize Particles ##############
	# The map is represented by a rectangle from (x1,y1) to (x2,y2)
	mapX1 = -10
	mapX2 = 10
	mapY1 = -10
	mapY2 = 10
	#Map size
	mapLX = mapX2 - mapX1
	mapLY = mapY2 - mapY1

	# number of particles
	N = 1000
	# Sparce the initial particles
	particles = []
	for i in range(N):
		p = Point32()
		# Position
		p.x = random.random() * mapLX + mapX1
		p.y = random.random() * mapLY + mapY1
		# weight
		p.z = 1.0 / N
		particles.append(p)
		


	# Draw the particles
	#area = np.pi * (150 * w)
	#plt.scatter(x, y, s=area, alpha=0.5)
	#plt.show()
	############ End Particle initialization #####

	# Wait while the world is totally spawned.
	rospy.sleep(5.0)
	print "wait for service"
	rospy.wait_for_service('/gazebo/get_model_state')
	getPosition = rospy.ServiceProxy('/gazebo/get_model_state', gazebo_msgs.srv.GetModelState)

	######## Control Loop ###########
	print "Start!"
	while not rospy.is_shutdown():
		# Robot position
		try:			
			resp = getPosition(robotName,"world")
			print "position [",resp.pose.position.x,",", resp.pose.position.y, "]"

		except rospy.ServiceException, e:
			print "Service call failed: %s" %e
		# TODO Get info from other robots.

		# Publish particles
		msg_parts = Polygon()
		msg_parts.points = particles
		partPub.publish(msg_parts)

		# Control
		vel = Twist()
		vel.linear.x = 1.00
		vel.angular.z = -1.50		
		velPub.publish(vel)

		rospy.sleep(0.50)


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
