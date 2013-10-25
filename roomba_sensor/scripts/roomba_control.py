#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from math import sqrt

import random

# Gazebo
import gazebo_msgs.srv

import copy

def resample(particles):
	print "resampling..."
	# Normalize Z
	sum = 0
	for p in particles:
		sum = sum + p.z
	print sum
	for p in particles:
		p.z = p.z / sum

	sum = 0
	for p in particles:
		sum = sum + p.z
	print "1=", sum

	
	# resampled particles
	resampledPrs = []
	for i in range(len(particles)):
		ran = random.random()
		s = 0
		for p in particles:
			s = p.z + s			
			if ran < s:
				resampledPrs.append(copy.deepcopy(p))
				break

	# Normalize resampled particles
	print "numRes=", len(resampledPrs)
	sum = 0
	for p in resampledPrs:
		sum = sum + p.z
		#print p.z
	#print "--"
	for i in range(len(resampledPrs)):		
		resampledPrs[i].z = resampledPrs[i].z / sum
		#print p.z

	#sum = 0
	#for i in range(len(resampledPrs)):	
	#	sum = sum + resampledPrs[i].z
	#print "new1=",sum,"\n"


	return resampledPrs


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
	partPub = rospy.Publisher("particles", Polygon)


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
	N = 10000
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
		

	############ End Particle initialization #####

	# Wait while the world is totally spawned.
	#rospy.sleep(5.0)
	print "wait for service"
	rospy.wait_for_service('/gazebo/get_model_state')
	getPosition = rospy.ServiceProxy('/gazebo/get_model_state', gazebo_msgs.srv.GetModelState)

	######## Control Loop ###########
	print "Start!"
	while not rospy.is_shutdown():
		# Robot position
		robotX = float("inf")
		robotY = float("inf")
		try:			
			resp = getPosition(robotName,"world")
			robotX = resp.pose.position.x
			robotY = resp.pose.position.y
			#print "position [",resp.pose.position.x,",", resp.pose.position.y, "]"

		except rospy.ServiceException, e:
			print "Service call failed: %s" %e
		# TODO Get info from other robots.

		# Move the particles
		for p in particles:
			mov = 0.4
			p.x = p.x + mov * random.random() - mov / 2.0
			p.y = p.y + mov * random.random() - mov / 2.0
		
		# Update particles
		for p in particles:
			# If the particles in the robot area.
			r = 1
			if sqrt((robotX - p.x)**2 + (robotY - p.y)**2) < r:
				p.z = p.z * 0.1
				#print "lugar errado da particula",p.z
			if p.x > mapX2 or p.x < mapX1 or p.y > mapY2 or p.y < mapY1:
				p.z = p.z * 0.1

			# TODO If the anomaly was sensed

		# TODO resampling
		particles = resample(particles)
		
		

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
