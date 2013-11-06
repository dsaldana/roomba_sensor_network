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
from roomba_sensor.roomba import RoombaGazebo
from roomba_sensor.particle_filter import ParticleFilter

# Map configuration
from roomba_sensor.map import *



sensedValue = 0

robotName = "r0"



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

def validateIndex(ni, nj, grid):
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

	# Goal Navigator
	navPub = rospy.Publisher("/" + robotName + "/goal", Point32)

	########## Initialize Particles ##############

	pf = ParticleFilter()
		

	############ End Particle initialization #####

	# Object to get information from Gazebo
	robot = RoombaGazebo(robotName)
	


	######## Control Loop ###########
	print "Start!"
	while not rospy.is_shutdown():
		# Get robot position from gazebo
		[robotX, robotY, robotT] = robot.getPosition()

		# Camera position
		[camX, camY, camT] = robot.getSensorPosition()

		
		print "robot", [robotX,robotY], " cam", [camX, camY]," sv=", sensedValue

		# Send the info to other robots.
		smsg = SensedValue()
		smsg.x = camX
		smsg.y = camY
		smsg.robot_id = robotName
		smsg.value = sensedValue
		sensorPub.publish(smsg)

		# Move the particles simulation the anomaly's dynamics
		pf.move_particles()
	


		

		pf.update_particles([[camX, camY, sensedValue]])
		# # Update particles
		# for p in particles:
		# 	# If the particles in the robot area.
		# 	r = 0.5 # radio to cover
		# 	if sqrt((camX - p.x)**2 + (camY - p.y)**2) < r:
		# 		if(sensedValue == 0):
		# 			p.z *= 0.1
		# 		else:
		# 			# If the anomaly was sensed
		# 			p.z *= 1.1 * sensedValue 
				
		# 		#print "lugar errado da particula",p.z
		# 	if p.x > mapX2 or p.x < mapX1 or p.y > mapY2 or p.y < mapY1:
		# 		p.z = p.z * 0.1

		grid = pf.particles_in_grid()
			

		# Resampling
		pf.resample()
		
		#for r in grid:
		#	print r
		

		# Publish particles
		msg_parts = Polygon()
		msg_parts.points = pf.particles
		partPub.publish(msg_parts)

		##### Plan in grid ####
		F = [[-1 for i in xrange(gm)] for j in xrange(gn)]
		D = [[-1 for i in xrange(gm)] for j in xrange(gn)]

		# sensor position in grid
		# print [camY,mapY1, gdy]
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
			
		maxi = -1
		maxj = -1
		maxv = -1					
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

		# print "sensor", [spi, spj]," target=",[maxi, maxj]
		#for c in F:	
		#	print c
		
		# Grid to coordinates
		targetX =  mapX1 + gdx * maxj + gdx / 2
		targetY =  mapY1 + gdy * maxi + gdy / 2


		#[targetX, targetY] = [3.0, 3.0]

		# Control
		theta = atan((camY-targetY) / (camX - targetX))
		print "from ",[camX, camY]," to ", [targetX, targetY]," vz=", degrees(theta-camY)," dif=", (camY-theta)

		# TODO control states: explore, track, and
		# vel = Twist()
		# vel.linear.x = 0
		# vel.angular.z = (camY-theta)/10
		#velPub.publish(vel)
		p = Point32()
		# Grid to cartesian plane.
		p.x = targetX
		p.y = targetY

		navPub.publish(p)

		rospy.sleep(0.50)


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
