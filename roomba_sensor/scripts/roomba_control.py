#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
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



# Sensed value by the camera
sensedValue = 0
# How many white pixels in the left
sensedLeft = 0
# How many white pixels in the right
sensedRight = 0

robotName = "r0"

robot_msgs = {}

# Callback for robot communication.
def robot_comm(msg):
	# Message from the same robot
	#if (msg.robot_id == robotName):
	#	return
	# Update a list of values
	robot_msgs[msg.robot_id] = msg
	

# Callback for camera sensor.
def img_callback(img):
	global sensedValue	
	global sensedLeft
	global sensedRight
	#OpenCV matrix
	mat = CvBridge().imgmsg_to_cv(img, "mono8")
	
	# How many white pixels in the left
	pl = 0
	# How many white pixels in the right
	pr = 0

	threshold_value = 230

	for i in [int(mat.rows / 2)]:
		for j in xrange(mat.cols/2):
			if(mat[i, j] > threshold_value):
				pl += 1
		for j in xrange(mat.cols/2, mat.cols):				
			if(mat[i, j] > threshold_value):
				pr += 1
	
	total = mat.rows * mat.cols * 1.0	
	sensedValue = (pl + pr) / total
	sensedLeft = (pl * 2.0) / (mat.rows)
	sensedRight = (pr * 2.0) / (mat.rows)
	
# Validate if one position in grid is valid.
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
	rospy.Subscriber("/robotCom", SensedValue, robot_comm)
	# Sensor's publisher
	sensorPub = rospy.Publisher("/robotCom", SensedValue)

	# Goal Navigator
	navPub = rospy.Publisher("/" + robotName + "/goal", Point32)
	# Tracker navigator
	trackPub = rospy.Publisher("/" + robotName + "/tracking", Float32)

	# Initialize Particles 
	pf = ParticleFilter()		

	# Object to get information from Gazebo
	robot = RoombaGazebo(robotName)
	


	######## Control Loop ###########
	print "Start!"
	while not rospy.is_shutdown():
		# Get robot position from gazebo
		[robotX, robotY, robotT] = robot.getPosition()

		# Camera position
		[camX, camY, camT] = robot.getSensorPosition()
				

		# Send the info to other robots.
		smsg = SensedValue()
		smsg.x = camX
		smsg.y = camY
		smsg.theta = camT
		smsg.robot_id = robotName
		smsg.value = sensedValue
		sensorPub.publish(smsg)

		# Particle filter: move the particles for simulating the anomaly's dynamics
		pf.move_particles()
	
		
		# Sensed values
		samples = []
		for msg in robot_msgs.values():
			samples.append([msg.x, msg.y,  msg.theta, msg.value])			
		
		# Particle filter: updade based on sensor value.
		pf.update_particles(samples)			

		# Particle filter: Resampling.
		pf.resample()
		
		# Publish particles
		msg_parts = Polygon()
		msg_parts.points = pf.particles
		partPub.publish(msg_parts)


		##### Plan in grid ####
		# Get a matrix with the number of particles for each cell.
		grid = pf.particles_in_grid()

		# Force matrix
		F = [[-1 for i in xrange(gm)] for j in xrange(gn)]
		# Distance matrix
		D = [[-1 for i in xrange(gm)] for j in xrange(gn)]

		# Convert sensor position to grid cell
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



		# Where to navigate
		goalX = None
		goalY = None

		######## Exploring #############
		if (sensedValue == 0):
			# Planning: Bread First Search #
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
							F[ni][nj] = grid[ni][nj] * exp( - 0.1 * D[ni][nj])						
							l.append([ni, nj])
							#TODO take into acount the other robots.
			
			# Find maximum force in grid
			maxi = -1
			maxj = -1
			maxv = -1					

			mvs = [[spi - 1, spj], [spi,spj+1], [spi, spj-1], [spi+1,spj]]
			for [i,j] in mvs:
				if validateIndex(i, j, grid) and F[i][j] > maxv:
					maxi = i
					maxj = j
					maxv = F[i][j]


			
			# Grid position to continuous coordinates
			goalX =  mapX1 + gdx * maxj + gdx / 2
			goalY =  mapY1 + gdy * maxi + gdy / 2

			# Publish goal to navigate
			p = Point32()
			p.x = goalX
			p.y = goalY
			navPub.publish(p)

		else:
			######## Tracking ############			
			controlP = (sensedLeft - 1) + sensedRight
			print "l=", sensedLeft, " r=",sensedRight, " control=", controlP
			trackPub.publish(controlP)


		rospy.sleep(0.1)


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
