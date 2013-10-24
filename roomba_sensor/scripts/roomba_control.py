#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# For plotting
import numpy as np
import matplotlib.pyplot as plt


def run():
	# Node roombaControl
	rospy.init_node('roomba_control')

	# Robot's name is an argument
	robotName = rospy.get_param('~robot_name', 'Robot1')
	print robotName

	# Create the Publisher to control the robot.
	topicName = "/" + robotName + "/commands/velocity"
	velPub = rospy.Publisher(topicName, Twist)


	#TODO this code must be out of this file
	########## Initialize Particles ##############
	# The map is represented by a rectangle from (x1,y1) to (x2,y2)
	mapX1 = -40
	mapX2 = 40
	mapY1 = -30
	mapY2 = 30
	#Map size
	mapLX = mapX2-mapX1
	mapLY = mapY2-mapY1

	# number of particles
	N = 50
	# Sparce the initial particles
	x = np.random.rand(N) * mapLX + mapX1
	y = np.random.rand(N) * mapLY + mapY1
	# weights
	w = np.ones(N) / N

	# Draw the particles
	area = np.pi * (150 * w)
	plt.scatter(x, y, s=area, alpha=0.5)
	plt.show()
	############ End Particle initialization #####

	# Wait while the world is totally spawned.
	rospy.sleep(5.0)
	
	print "start"
	while not rospy.is_shutdown():
		vel = Twist()
		vel.linear.x = 1.00
		vel.angular.z = -1.50		
		#velPub.publish(vel)

		rospy.sleep(0.50)


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
