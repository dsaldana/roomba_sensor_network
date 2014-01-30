#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32

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

# Clustering
from scipy.cluster.vq import kmeans2


import random

# Window size
width, height = 750, 600

# Window	
pygame.init() 
window = pygame.display.set_mode((width, height), HWSURFACE | DOUBLEBUF | RESIZABLE) 
particles = None

goal = Point32()
goal.x, goal.y = 0.0 ,0.0

draw_clusters = rospy.get_param('/draw_clusters', False)
draw_particles1 = rospy.get_param('/draw_particles', False)
draw_anomaly = rospy.get_param('/draw_anomaly', False)
draw_path1 = rospy.get_param('/draw_path', False)
draw_ellipse = rospy.get_param('/draw_ellipse', False)
draw_grid = rospy.get_param('/draw_grid', False)

# set of points with robots' paths
paths = {}

def goal_callback(point):
	global goal
	goal = point
	print "new goal ", [goal.x, goal.y]

def convert_axis_p(p):
	x2,y2 = convertAxis[p[0], p[1]]
	return [x2, y2]

def convertAxis(x,y):
	x2 = mx + (x - mapX1) * ax / mapLX
	y2 = my + (-y - mapY1) * ay / mapLY
	return x2, y2

def draw_robot(robotX, robotY, robotT, color=(0, 0, 170)):
	x2,y2 = convertAxis(robotX, robotY)

	# Robot radio
	rd = int((mx + my) / 8)
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
	

def save_path(robot_id, robotX, robotY, robotT):
	global paths
	p = Point32()
	p.x, p.y, p.z = robotX, robotY, robotT

	if not robot_id in paths:
		paths[robot_id] = []

	paths[robot_id].append(p)

def draw_path(points, color=(255,0,0)):
	if len(points) < 2:
		return

	lp = convertAxis(points[0].x, points[0].y)

	for i in range(1, len(points)):
		# end point
		ep = convertAxis(points[i].x, points[i].y)
		# draw line
		pygame.draw.line(window, color, lp, ep)

		lp = ep 

def draw_points(points, color=(255,0,0)):

	if len(points) == 0:
		return

	x, y = [], []
	for p in points:
		x2 ,y2= convertAxis(p.x, p.y)
		x.append(x2)
		y.append(y2)
	
	
	if not draw_clusters:
		for i in range(len(x)):
			pygame.draw.circle(window, color, (int(x[i]), int(y[i])), 2, 0)
	else: 
		k_groups = 9

		# let scipy do its magic (k_groups)
		res, idx = kmeans2(np.array(zip(x, y)), 
			np.array([[1,1],[-1,-1],[2,2]]))
		
		# Categorize in k groups and compute
		# new centroids
		np.random.seed(1)
		try:
			
			cents, idx = kmeans2(np.array(zip(x, y)), k_groups)
			
			np.savetxt("cents.csv", cents, delimiter=",")


			# convert groups to rbg 3-tuples.
			colors = ([([0,255,0],[255,0,0],[0,0,255],
				[0,100,100],[100,100,0],[100,0,100],
				[0,180,0],[185,0,0],[0,0,185])[i] for i in idx])

			for i in range(len(x)):
				pygame.draw.circle(window, colors[i], (int(x[i]), int(y[i])), 2, 0)

			for c in cents:
				pygame.draw.circle(window, [0,0,0], (int(c[0]), int(c[1])), 5, 0)		
		except Exception, e:
			print e
	return

def callback(particles_msg):
	global particles
	global new_msg
	particles = particles_msg
	new_msg = True

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

	ilines = range(gn + 1)
	jlines = range(gm + 1)

	if not draw_grid:
		ilines = [0, len(ilines)-1]
		jlines = [0, len(jlines)-1]

	# Draw rows
	color = (170, 170, 170)
	for i in ilines:
		hr = my + i * dy
		pygame.draw.line(window, color , (mx, hr ), (width - mx, hr))
	# Draw columns
	for j in jlines:
		hc = mx + j * dx
		pygame.draw.line(window, color, ( hc ,my), (hc, height - my))
	
	# Draw zeros
	if draw_grid:
		pygame.draw.line(window, (0, 0, 250), (mx / 2, my + dy * gn / 2),
			(width - mx / 2, my + dy * gn / 2))
		pygame.draw.line(window, (0, 0, 250), 
			(mx + dx * gm / 2, my / 2),
			(mx + dx * gm / 2, height - my / 2))
	
	# path colors
	pcols = {"Robot4" : (0, 0, 200), "Robot2":(0, 200, 200),
		"Robot3":(0, 0, 200), "Robot3":(200, 0, 200)}

	# Draw each robot path
	if draw_path1:
		for k, rp in paths.iteritems():
			color = (0, 0, 200)
			if k in pcols:
				color = pcols[k]
			draw_path(rp, color)

	# Draw particles
	if draw_particles1:
		draw_points(particles.particles,(0, 200, 0))
	
	# Draw anomaly points
	if draw_anomaly:
		draw_points(particles.anomaly)


	# # Write particles in a CSV file
	u = []
	# for p in particles.particles:
	# 	u.append([p.x, p.y])

	# np.savetxt("foo.csv", u, delimiter=",")	

	# if "Robot1" in paths and not particles.anomaly:
	# 	print "Writing"
	# 	path_size = len(paths["Robot1"])		

	# 	for i in range(1, path_size):
	# 		l = []
	# 		for rname in paths.keys():
	# 			if len(paths[rname]) > i:
	# 				l.append(paths[rname][i].x)
	# 				l.append(paths[rname][i].y)
	# 				l.append(paths[rname][i].z)
	# 		u.append(l)

		
	# 	#Write path		
	# 	try:			
	# 		np.savetxt("path.csv", np.array(u) , delimiter=",")
	# 	except Exception, e:
	# 		print e
	# 		print u

	# ## End Write

	# Fit the anomaly to an ellipse

	if len(particles.anomaly) > 0 and draw_ellipse:				
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
		
		
		try:
			center, axes, phi  = fit_ellipse(x, y)

			if max(axes) > 100:
				rospy.logerr("Elipse fuera de rango")
			else:
				#print "drawing ellipse", [center, axes, degrees(phi)]
				pe_x, pe_y  = ellipse_points(center, axes, phi, n = width)
				
				for i in range(len(pe_x)):
					ex, ey = pe_x[i], pe_y[i]
					ex, ey = convertAxis(ex, ey)
					pygame.draw.circle(window, (0, 0, 200), (int(ex), int(ey)) , int(1), 0)

				ecx, ecy = convertAxis(center[0], center[1])	
				pygame.draw.circle(window, (0, 0, 200), (int(ecx), int(ecy)) , int(5), 0)
		except Exception, e:
			print e
		
		

	# Draw the other robots
	for orobot in particles.orobots:
		robotX, robotY, robotT = orobot.x, orobot.y, orobot.z
		draw_robot(robotX, robotY, robotT, (150, 150, 150))
		if draw_path1 and new_msg:
			save_path(orobot.robot_id, robotX, robotY, robotT)

	# robot position (just for name reduction)
	robotX, robotY, robotT = particles.mrobot.x , particles.mrobot.y, particles.mrobot.z

	# Draw the target
	#gx, gy = convertAxis(goal.x, goal.y)
	#pygame.draw.line(window, color , convertAxis(robotX, robotY), (gx, gy),5)

	# Draw main robot	
	draw_robot(robotX, robotY, robotT)


	pygame.display.flip() 




def run():
	global new_msg
	new_msg = False
	# Node roombaControl
	rospy.init_node('particle_drawer')		

	# Suscribe
	robot_name = rospy.get_param('~robot_name', 'Robot1')
	rospy.Subscriber("/" + robot_name + "/particles", Particle, callback,  queue_size = 1)
	rospy.Subscriber("/" + robot_name + "/goal", Point32, goal_callback,  queue_size = 1)			

	# Window
	pygame.RESIZABLE = True
	while not rospy.is_shutdown():
		rospy.sleep(0.02)
		# PyGame events
		for event in pygame.event.get(): 
			if event.type == pygame.QUIT: 
				sys.exit(0) 
			elif event.type == VIDEORESIZE:
				window = pygame.display.set_mode(event.dict['size'],
					HWSURFACE | DOUBLEBUF | RESIZABLE)
			#else: 
				#print event 
		# Draw the particles
		if particles is not None:
			draw_particles()
		new_msg = False
		


if __name__ == '__main__':
	#global robot
	try:
		run()		
	except rospy.ROSInterruptException:
		sys.exit(0)
		pass
