#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

# Image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


from math import sqrt
from math import degrees
from math import cos
from math import sin
import random

# Gazebo
import gazebo_msgs.srv

import copy

sensedValue = 0

def resample(particles):
	print "resampling..."
	
	N = len(particles)
	index =  int(random.random() * N)
	
	# weights
	w = [0] * N
	for i in range(N):
		w[i] = particles[i].z

	# resampled particles
	rp = [0] * N

	# resample
	beta = 0.0
	mx = max (w)
	for i in range(N):
		beta += random.random() * 2.0 * mx
		while beta > w[index]:
			beta -= w[index]
			index = (index + 1) % N
		rp[i] = copy.deepcopy(particles[index])	

	for p in rp:
		p.z /=mx
	return rp


def img_callback(img):
	global sensedValue
	#print "image", img.height, "x", img.width," ", len(img.data)," enc=",img.encoding, " step=",img.step
	#OpenCV matrix
	mat = CvBridge().imgmsg_to_cv(img, "mono8")
	
	# How many white pixels in the left
	pl = 0
	# How many white pixels in the right
	pr = 0

	for i in xrange(mat.rows):
		for j in xrange(mat.cols/2):
			if(mat[i, j] > 230):
				pl += 1
		for j in xrange(mat.cols/2, mat.cols):				
			if(mat[i, j] > 230):
				pr += 1
	
	total = mat.rows * mat.cols * 1.0
	#print "blancos ", pl/total , ",", pr/total
	
	sensedValue = (pl+pr) / total
	print "sensedValue=", sensedValue

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
	
	# Camera
	topicName = "/" + robotName + "/front_cam/camera/image"
	image_sub = rospy.Subscriber(topicName, Image, img_callback)

	########## Initialize Particles ##############
	# The map is represented by a rectangle from (x1,y1) to (x2,y2)
	mapX1 = -5
	mapX2 = 5
	mapY1 = -5
	mapY2 = 5
	#Map size
	mapLX = mapX2 - mapX1
	mapLY = mapY2 - mapY1

	# number of particles
	N = 5000
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
			robotT = resp.pose.orientation.z
			#print "position ", degrees(resp.pose.orientation.z)
		except rospy.ServiceException, e:
			print "Service call to gazebo failed: %s" %e

		# Camera area
		d = 0.5
		camX = robotX + d * cos(robotT)
		camY = robotY + d * sin(robotT)
		#print "robot", [robotX,robotY], " cam", [camX, camY]
		print sensedValue

		# TODO Get info from other robots.

		# Move the particles
		for p in particles:
			mov = 0.4
			p.x = p.x + mov * random.random() - mov / 2.0
			p.y = p.y + mov * random.random() - mov / 2.0
		
		# Update particles
		for p in particles:
			# If the particles in the robot area.
			r = 0.5 # radio to cover
			#if sqrt((camX - p.x)**2 + (camY - p.y)**2) < r:
				#p.z = 2* p.z * sensedValue + 0.01
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
		vel.linear.x = 0.500
		vel.angular.z = -0.750		
		#velPub.publish(vel)

		rospy.sleep(0.50)


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
