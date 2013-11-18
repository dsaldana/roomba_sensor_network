#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
# Image
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32


# OpenCV
from cv_bridge import CvBridge, CvBridgeError
import cv

# Math
from math import *
import random

from roomba_sensor.roomba import RoombaGazebo
from roomba_sensor.particle_filter import ParticleFilter
from roomba_sensor.grid_util import bread_first_search
from roomba_sensor.grid_util import validate_index
from roomba_sensor.grid_util import coords_to_grid
from roomba_sensor.grid_util import grid_to_coords

# Map configuration
from roomba_sensor.map import *
from roomba_comm.msg import SensedValue

# Numpy
import numpy as np



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
	mat = CvBridge().imgmsg_to_cv(img, "rgb8")
	# Extract red channel
	red_channel = cv.CreateImage(cv.GetSize(mat), 8, 1)
	cv.Split(mat, red_channel, None, None, None)
	
	
	# How many white pixels in the left
	pl = 0
	# How many white pixels in the right
	pr = 0

	threshold_value = 230

	# Conunt the pixels in the middel row of the image.
	for i in [int(mat.rows / 2)]:
		for j in xrange(mat.cols / 2):
			if(red_channel[i, j] > threshold_value):
				pl += 1
		for j in xrange(mat.cols/2, mat.cols):				
			if(red_channel[i, j] > threshold_value):
				pr += 1
	
	total = mat.rows * mat.cols * 1.0	
	sensedValue = (pl + pr) / total
	sensedLeft = (pl * 2.0) / (mat.rows)
	sensedRight = (pr * 2.0) / (mat.rows)
	
	

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
	
	# Explore flag. False for tracking
	explore = True

	# Time for tracking without sensing anomaly.
	max_tracking_time = 5
	# last time that an anomaly was detected
	last_time_anomaly = 0


	######## Control Loop
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

		# Convert sensor position to grid cell
		spi, spj = coords_to_grid(camX, camY)


		# Where to navigate
		goalX = None
		goalY = None

		# The flag for exploring change only if  the robot does not
		# sense an anomaly in a n seconds (n = max_tracking_time).
		if sensedValue > 0:
			explore = False
			last_time_anomaly = rospy.get_rostime().secs
		else:
			if rospy.get_rostime().secs - last_time_anomaly > max_tracking_time:
				explore = True

		
		######## Exploring
		if (explore):
			npgrid = np.array(grid)

			#### Planning: Bread First Search 
			# Distance matrix
			D = np.array(bread_first_search(spi, spj, grid))			
			#rospy.logerr("D")
			#rospy.logerr(D)
			
			#TODO take into acount the other robots.
			DRT = np.zeros((gn,gm))
			for r in robot_msgs.values():
				if (r.robot_id == robotName):
					continue
				# Distances from robot
				ri,rj = coords_to_grid(r.x, r.y)
				#rospy.logerr("Robot position")
				#rospy.logerr([ri,rj])
				BFS = bread_first_search(ri, rj, grid)
				#rospy.logerr(BFS)
				u = 0.5 * np.max(npgrid) * np.exp(-np.array(BFS))
				DRT += u
				
				
			#rospy.logerr("DRT")
			#rospy.logerr(DRT)
			# Number of robots
			n_robots = len(robot_msgs.values())
			
			# Force for one robot.
			F = npgrid * np.exp(-0.1 * D)
			#rospy.logerr("F")
			#rospy.logerr(F)
			
			# Force from robot location to every cell.
			if n_robots > 1:
				#F -=  npgrid * np.exp(- 0.5 * DRT / (n_robots - 1))
				F -= DRT / (n_robots - 1)

			# Find maximum force in grid
			maxi, maxj = np.unravel_index(F.argmax(), F.shape)


			
			# Grid position to continuous coordinates. 
			# goal points to the center point in the cell.
			goalX , goalY = grid_to_coords(maxi,maxj)

			# Publish goal to navigate
			p = Point32()
			p.x = goalX
			p.y = goalY
			navPub.publish(p)

		else:
			#### Tracking
			controlP = (sensedLeft - 1) + sensedRight
			print "l=", sensedLeft, " r=",sensedRight, " control=", controlP
			trackPub.publish(controlP)


		rospy.sleep(0.1)


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
