#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from roomba_comm.msg import SensedValue

# Image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


from math import *
import random

# Gazebo
import gazebo_msgs.srv

import copy

sensedValue = 0

robotName = "r0"

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

def robot_comm(msg):
	# Message from the same robot
	if (msg.robot_id == robotName):
		return
	# Update a list of values
	print msg


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
	#print "sensedValue=", sensedValue

def validateIndex(ni,nj,grid):
	if(ni < 0 or ni >= len(grid)):
		return False
	if (nj < 0 or nj >= len(grid[0])):
		return False
	return True
	

def run():
	######### Initialization ##################
	# Node roombaControl
	rospy.init_node('roomba_control')

	# Robot's name is an argument
	global robotName
	robotName = rospy.get_param('~robot_name', 'Robot1')
	#print robotName

	# Create the Publisher to control the robot.
	topicName = "/" + robotName + "/commands/velocity"
	velPub = rospy.Publisher(topicName, Twist)

	# Create a publisher for the particles
	partPub = rospy.Publisher("particles", Polygon)
	
	# Camera
	topicName = "/" + robotName + "/front_cam/camera/image"
	image_sub = rospy.Subscriber(topicName, Image, img_callback)

	# Robot communication
	# Subscriber for robot communication
	rospy.Subscriber("robotCom", SensedValue, robot_comm)
	# Sensor's publisher
	sensorPub = rospy.Publisher("robotCom", SensedValue)

	########## Initialize Particles ##############
	# The map is represented by a rectangle from (x1,y1) to (x2,y2)
	mapX1 = -5.0
	mapX2 = 5.0
	mapY1 = -5.0
	mapY2 = 5.0
	#Map size
	mapLX = mapX2 - mapX1
	mapLY = mapY2 - mapY1
	# Grid size		
	gn = 20 # Number of rows
	gdx = mapLX / gn # delta x
	gm = 20 # Number of columns
	gdy = mapLY / gm # delta y

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
		# Get robot position from gazebo
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
		print "robot", [robotX,robotY], " cam", [camX, camY]," sv=",sensedValue

		# Send the info to other robots.
		smsg = SensedValue()
		smsg.x = camX
		smsg.y = camY
		smsg.robot_id = robotName
		smsg.value = sensedValue
		sensorPub.publish(smsg)

		# Move the particles
		for p in particles:
			mov = 0.2
			# Normal fuction
			p.x = random.normalvariate(p.x, mov)
			p.y = random.normalvariate(p.y, mov)
	

		# Particle position in grid n rows and m columns
		grid=[[0 for x in xrange(gm)] for x in xrange(gn)]
		
		# Update particles
		for p in particles:
			# If the particles in the robot area.
			r = 0.5 # radio to cover
			if sqrt((camX - p.x)**2 + (camY - p.y)**2) < r:
				if(sensedValue == 0):
					p.z *= 0.1
				else:
					# If the anomaly was sensed
					p.z *= 1.1 * sensedValue 
				
				#print "lugar errado da particula",p.z
			if p.x > mapX2 or p.x < mapX1 or p.y > mapY2 or p.y < mapY1:
				p.z = p.z * 0.1

			# Particle in grid
			if(p.x > mapX1 and p.x < mapX2 and p.y > mapY1 and p.y < mapY2):
				i = int((p.y- mapY1) / gdy)
				j = int((p.x- mapX1) / gdx)
				#print "i,j",[i,j], ",",[p.y,p.x]
				grid[i][j] += 1
			

		# Resampling
		particles = resample(particles)
		
		#for r in grid:
		#	print r
		

		# Publish particles
		msg_parts = Polygon()
		msg_parts.points = particles
		partPub.publish(msg_parts)

		##### Plan in grid ####
		F = [[-1 for i in xrange(gm)] for j in xrange(gn)]
		D = [[-1 for i in xrange(gm)] for j in xrange(gn)]

		# sensor position in grid
		print [camY,mapY1, gdy]
		spi = int((camY - mapY1) / gdy)
		spj = int((camX - mapX1) / gdx)
		
		if(spi > gm-1):
			spi = gm-1
		if(spj > gn-1):
			spj = gn-1
		if(spi < 0):
			spi = 0
		if (spj < 0):
			spj = 0

		# BFS		
		D[spi][spj] = 0
		
		l = []
		# The current position
		l.append([spi, spj])
	
		while (len(l) > 0):			
			[i,j] = l.pop(0)
			# Possible movements[up, right, left, down]
			mvs = [[i - 1, j], [i,j+1], [i, j-1], [i+1,j]]

			for [ni,nj] in mvs:
				if validateIndex(ni,nj,grid):
					# Not visited node
					if (D[ni][nj] < 0):
						D[ni][nj] = D[i][j] + 1
						F[ni][nj] = grid[ni][nj] * exp( -D[ni][nj])						
						l.append([ni, nj])
			
		maxi=-1
		maxj=-1
		maxv=-1					
		#for i in range(len(F)):
		#	for j in range(len(F[0])):
		#		if(F[i][j] > maxv):
		#			maxi = i
		#			maxj = j
		#			maxv = F[i][j]
		mvs = [[spi - 1, spj], [spi,spj+1], [spi, spj-1], [spi+1,spj]]
		for [i,j] in mvs:
			if validateIndex(i, j, grid) and F[i][j] > maxv:
				maxi = i
				maxj = j
				maxv = F[i][j]

		print "sensor", [spi, spj]," target=",[maxi, maxj]
		#for c in F:	
		#	print c
		
		# Grid to coordinates
		targetX =  mapX1 + gdx * maxj + gdx / 2
		targetY =  mapY1 + gdy * maxi + gdy / 2


		[targetX, targetY] = [3,3]

		# Control
		theta = atan((camY-targetY) / (camX-targetX))
		print "from ",[camX, camY]," to ", [targetX, targetY]," vz=", degrees(theta-camY)," dif=", (camY-theta)

		# TODO control states: explore, track, and
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
