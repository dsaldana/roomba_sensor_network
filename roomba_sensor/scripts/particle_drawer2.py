#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from roomba_comm.msg import Particle
from roomba_comm.msg import PointR
from roomba_sensor.ellipse import fit_ellipse
from roomba_sensor.ellipse import ellipse_box
from roomba_sensor.ellipse import ellipse_points

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
particles = None

def convert_axis_p(p):
	x2,y2 = convertAxis[p[0], p[1]]
	return [x2, y2]

def convertAxis(x,y):
	x2 = mx + (x - mapX1) * ax / mapLX
	y2 = my + (-y - mapY1) * ay / mapLY
	return x2,y2

def draw_robot(robotX, robotY, robotT, color=(0, 0, 170)):
	x2,y2 = convertAxis(robotX, robotY)

	# Robot radio
	rd = int((mx + my)/ 12)
	if rd <1:
		rd = 1
	# Line for direction
	linew = int((mx + my)/ 40)
	if linew < 1:
		linew = 1
		
	pygame.draw.circle(window, color, (int(x2), int(y2)), rd, 0)
	pygame.draw.line(window, (255, 248, 0), 
		(x2, y2),
		(x2 + rd * cos(robotT), y2 - rd * sin(robotT)), linew)
	

def draw_points(points, color=(255,0,0)):	
	for p in points:
		x2 ,y2= convertAxis(p.x, p.y)
		pygame.draw.circle(window, color, (int(x2), int(y2)), 2, 0)
		

def callback(particles_msg):
	global particles
	particles = particles_msg

def draw_particles():
	# Margin
	global mx
	global my
	# Grid constats
	global ax
	global ay
	global dx
	global dy
	global particles

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
	
	# Draw anomaly
	draw_points(particles.anomaly)

	particles
	# Fit the anomaly to an ellipse
	if len(particles.anomaly) > 0:				
		######## Draw ellipse aproximation
		# Convert PointR to numpy
		NE = len(particles.anomaly)
		#NE = 40
		x = [0] * NE
		y = [0] * NE
		#for i in range(NE):
		for i in range(0, NE):
			j = len(particles.anomaly)-NE -1 + i
			x[i],y[i] = particles.anomaly[j].x, particles.anomaly[j].y	
		
		
		center, axes, phi  = fit_ellipse(x, y)

		try:
				
			if max(axes) > 100:
				rospy.logerr("Elipse fuera de rango")
			else:
				print "drawing ellipse", [center, axes, degrees(phi)]
				pe_x, pe_y  = ellipse_points(center, axes, phi, n=width)
				
				for i in range(len(pe_x)):
					ex, ey = pe_x[i], pe_y[i]
					ex, ey = convertAxis(ex, ey)
					pygame.draw.circle(window, (200,0,0), (int(ex), int(ey)) , int(1), 0)

				ecx, ecy = convertAxis(center[0], center[1])	
				pygame.draw.circle(window, (128,0,0), (int(ecx), int(ecy)) , int(5), 0)
		except Exception, e:
			print e
		
			

	# Draw the other robots
	for orobot in particles.orobots:
		robotX, robotY, robotT = orobot.x, orobot.y, orobot.z
		draw_robot(robotX, robotY, robotT, (150,150,150))


	# Draw main robot
	robotX, robotY, robotT = particles.mrobot.x , particles.mrobot.y, particles.mrobot.z
	draw_robot(robotX, robotY, robotT)


	pygame.display.flip() 




def run():

	# Node roombaControl
	rospy.init_node('particle_drawer')		

	# Suscribe
	robotName = rospy.get_param('~robot_name', 'Robot1')
	rospy.Subscriber("/" + robotName + "/particles", Particle, callback)
					

	# Window
	pygame.RESIZABLE = True
	while not rospy.is_shutdown():
		rospy.sleep(0.1)
		# PyGame events
		for event in pygame.event.get(): 
			if event.type == pygame.QUIT: 
				sys.exit(0) 
			elif event.type==VIDEORESIZE:
				window = pygame.display.set_mode(event.dict['size'],HWSURFACE|DOUBLEBUF|RESIZABLE)
			#else: 
				#print event 
		# Draw the particles
		if particles is not None:
			draw_particles()
		
		


if __name__ == '__main__':
	#global robot
	try:
		run()		
	except rospy.ROSInterruptException:
		sys.exit(0)
		pass
