#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from roomba_comm.msg import Particle
from roomba_comm.msg import PointR

# For plotting
import numpy as np
import sys
import pygame
from pygame.locals import *

import time

# Map configuration
from roomba_sensor.map import *
from math import *

# Window size
width, height = 750, 600

# Window	
pygame.init() 
window = pygame.display.set_mode((width, height),HWSURFACE|DOUBLEBUF|RESIZABLE) 

	
def draw_robot(robotX, robotY, robotT, color=(0, 0, 170)):
	x2 = mx + (robotX - mapX1) * ax / mapLX
	y2 = my + (-robotY - mapY1) * ay / mapLY
	rd = 8
	pygame.draw.circle(window, color, (int(x2), int(y2)), rd, 0)
	pygame.draw.line(window, (255, 248, 0), 
		(x2, y2),
		(x2 + rd * cos(robotT), y2 - rd * sin(robotT)), 2)
	

def draw_points(points, color=(255,0,0)):
	
	for p in points:
		x2 = mx + (p.x - mapX1) * ax / mapLX
		y2 = my + (- p.y - mapY1) * ay / mapLY
		pygame.draw.circle(window, color, (int(x2), int(y2)), 2, 0)
		

def callback(particles):
	# Margin
	global mx
	global my
	# Grid constats
	global ax
	global ay
	global dx
	global dy
	
	# Window size
	try:
		width, height =  window.get_size()
	except :
		return
	
	# Margin
	mx = width / 12.0
	my = height / 12.0
	# Grid constats
	ax = width - 2.0 * mx # area for x
	ay = height - 2.0 * my # area for y
	dx = ax / gm #Delta x
	dy = ay / gn #Delta y
	
	
	

		
	# Draw the canvas
	window.fill((255, 255, 255))	

	# Draw rows
	color = (170, 170, 170)
	for i in range(gn + 1):
		hr = my + i * dy
		pygame.draw.line(window, color , (mx, hr ), (width - mx, hr))
	# Draw columns
	for j in range(gm + 1):
		hc = mx + j * dx
		pygame.draw.line(window, color, ( hc ,my), (hc, height - my))
	
	# Draw zeros
	pygame.draw.line(window, (0, 0, 250), (mx / 2, my + dy * gn / 2),
		(width - mx / 2, my + dy * gn / 2))
	pygame.draw.line(window, (0, 0, 250), 
		(mx + dx * gm / 2, my / 2),
		(mx + dx * gm / 2, height - my / 2))
		
	# Draw particles
	draw_points(particles.particles,(0,200,0))
	
	# Draw particles
	draw_points(particles.anomaly)
	

	# Draw main robot
	robotX, robotY, robotT = particles.mrobot.x , particles.mrobot.y, particles.mrobot.z
	draw_robot(robotX, robotY, robotT)


	# Draw the other robots
	for orobot in particles.orobots:
		robotX, robotY, robotT = orobot.x, orobot.y, orobot.z
		draw_robot(robotX, robotY, robotT, (150,150,150))


	
	pygame.display.flip() 

if __name__ == '__main__':
	#global robot
	try:
		# Node roombaControl
		rospy.init_node('particle_drawer', anonymous=True)		

		# Suscribe
		robotName = rospy.get_param('~robot_name', 'Robot1')
		rospy.Subscriber("/" + robotName + "/particles", Particle, callback)
		
		# Object to get information from Gazebo
		#robot = RoombaGazebo(robotName)

		# Window
		pygame.RESIZABLE = True
		while True: 
			time.sleep(0.1)
			for event in pygame.event.get(): 
				if event.type == pygame.QUIT: 
					sys.exit(0) 
				elif event.type==VIDEORESIZE:
					window = pygame.display.set_mode(event.dict['size'],HWSURFACE|DOUBLEBUF|RESIZABLE)
				#else: 
					#print event 

	except rospy.ROSInterruptException:
		sys.exit(0)
		pass
