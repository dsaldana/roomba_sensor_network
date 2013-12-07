#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
# Image
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32
from roomba_comm.msg import Particle
from roomba_comm.msg import PointR


# OpenCV
from cv_bridge import CvBridge, CvBridgeError
import cv

# Math
from math import *
import random

from roomba_sensor.roomba import RoombaLocalization
from roomba_sensor.particle_filter import ParticleFilter
from roomba_sensor.grid_util import bread_first_search
from roomba_sensor.grid_util import validate_index
from roomba_sensor.grid_util import coords_to_grid
from roomba_sensor.grid_util import grid_to_coords
from roomba_sensor.grid_util import maximum_neightbor

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
	green_channel = cv.CreateImage(cv.GetSize(mat), 8, 1)
	blue_channel = cv.CreateImage(cv.GetSize(mat), 8, 1)
	cv.Split(mat, red_channel, green_channel, blue_channel, None)
	
	
	# How many white pixels in the left
	pl = 0
	# How many white pixels in the right
	pr = 0

	threshold_value = 160
	threshold_other = 160
	
	# Conunt the pixels in the middel row of the image.
	for i in [int(mat.rows / 2)]:
		for j in xrange(mat.cols / 2):
			if red_channel[i, j] > threshold_value and green_channel[i, j] < threshold_other and blue_channel[i, j] < threshold_other:
				pl = j
				
		for j in xrange(int(mat.cols / 2) + 1, mat.cols):				
			if red_channel[i, j] > threshold_value and green_channel[i, j] < threshold_other and blue_channel[i, j] < threshold_other:
				pr = j - int(mat.cols / 2)
	
	total = mat.rows * mat.cols * 1.0	
	sensedValue = (pl + pr) / total
	sensedLeft = (pl * 2.0) / (mat.rows)
	sensedRight = (pr * 2.0) / (mat.rows)
	
	#print [total, sensedValue, sensedLeft, sensedRight]
	

def run():	
	######### Initialization ##################
	# Node roombaControl
	rospy.init_node('roomba_control')

	# Robot's name is an argument
	global robotName
	robotName = rospy.get_param('~robot_name', 'Robot1')

	rospy.loginfo("Loading robot control.")

	# Create the Publisher to control the robot.
	topicName = "/" + robotName + "/commands/velocity"
	velPub = rospy.Publisher(topicName, Twist)
	

	# Create a publisher for the particles
	partPub = rospy.Publisher("/" + robotName + "/particles", Particle)
	
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

	## Robot Localization
	robot = RoombaLocalization(robotName)
	
	# Explore flag. False for tracking
	explore = True

	# Time for tracking without sensing anomaly.
	max_tracking_time = 5
	# last time that an anomaly was detected
	last_time_anomaly = 0

	anomaly_points = []
	######## Control Loop
	print "Start!"
	while not rospy.is_shutdown():
		# Get robot position from gazebo
		[robotX, robotY, robotT] = robot.get_position()		
		# Camera position
		[camX, camY, camT] = robot.get_sensor_position()

		# Send the info to other robots.
		smsg = SensedValue()
		smsg.x,smsg.y, smsg.theta  = camX, camY, camT
		smsg.rx,smsg.ry, smsg.rtheta  = robotX, robotY, robotT
		
		smsg.robot_id = robotName
		smsg.value = sensedValue
		sensorPub.publish(smsg)

		# Particle filter: move the particles for simulating the anomaly's dynamics
		pf.move_particles()
	
		
		# Sensed values
		samples = []
		orobots = []
		for msg in robot_msgs.values():
			samples.append([msg.x, msg.y,  msg.theta, msg.value])
			# Other robot positions
			orobot = PointR()
			orobot.x, orobot.y, orobot.z = msg.rx, msg.ry,  msg.rtheta
			orobots.append(orobot)
		
		# Particle filter: updade based on sensor value.
		pf.update_particles(samples)

		# Particle filter: Resampling.
		pf.resample()
		
		# Publish particles
		msg_parts = Particle()
		msg_parts.particles = pf.particles
		mrobot = PointR()
		mrobot.x, mrobot.y, mrobot.z = robotX, robotY, robotT
		msg_parts.mrobot = mrobot
		msg_parts.orobots = orobots
		msg_parts.anomaly = anomaly_points
		partPub.publish(msg_parts)


		##### Plan in grid ####
		# Get a matrix with the number of particles for each cell.
		grid = pf.particles_in_grid()

		# Convert sensor position to grid cell
		spi, spj = coords_to_grid(camX, camY)


		# Where to navigate
		goalX = None
		goalY = None

		#### Evaluate Sensed value
		# The flag for exploring change only if  the robot does not
		# sense an anomaly in a n seconds (n = max_tracking_time).
		if sensedValue > 0:
			explore = False
			last_time_anomaly = rospy.get_rostime().secs
			
			# new point with anomaly
			an = PointR()
			an.x, an.y = robotX, robotY
			anomaly_points.append(an)
		else:
			if rospy.get_rostime().secs - last_time_anomaly > max_tracking_time:
				explore = True

		
		######## Exploring
		if explore:
			npgrid = np.array(grid)

			#### Planning: Bread First Search 
			# Distance matrix to particles
			D = np.array(bread_first_search(spi, spj, grid))			

			
			# Take into acount the other robots.
			# It needs to be tested
			DRT = np.zeros((gm,gn))

			try:
				for r in robot_msgs.values():
					if (r.robot_id == robotName):
						continue
					# Distances from the other robot
					ri,rj = coords_to_grid(r.x, r.y)

					BFS = bread_first_search(ri, rj, grid)

					u = np.max(npgrid) * np.exp(-1 * np.array(BFS))
					DRT += u
			except Exception, e:
				rospy.logerr("Error integrating the data from other robots. " + str(e))
			
				
				
			# Number of robots
			n_robots = len(robot_msgs.values())
			
			# Force for one robot.
			F = npgrid * np.exp(-0.1 * D)

			
			# Force from robot location to every cell.
			if n_robots > 1:
				F -= DRT #/ (n_robots - 1)

			# Find maximum force in grid
			# TODO select the best in front of the robot or modify the BFS
			#maxi, maxj = np.unravel_index(F.argmax(), F.shape)			
			maxi, maxj = maximum_neightbor(spi, spj, F)			
			
			# Grid position to continuous coordinates. 
			# goal points to the center point in the cell.
			goalX , goalY = grid_to_coords(maxi, maxj)

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
