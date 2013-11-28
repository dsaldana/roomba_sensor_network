#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from roomba_comm.msg import Particle
from roomba_comm.msg import PointR


import cv2


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


###### ellipse
## FROM http://stackoverflow.com/questions/13635528/fit-a-ellipse-in-python-given-a-set-of-points-xi-xi-yi
import numpy.linalg as linalg
def fitEllipse(x,y):
    x = x[:,np.newaxis]
    y = y[:,np.newaxis]
    D =  np.hstack((x*x, x*y, y*y, x, y, np.ones_like(x)))
    S = np.dot(D.T,D)
    C = np.zeros([6,6])
    C[0,2] = C[2,0] = 2; C[1,1] = -1
    E, V =  linalg.eig(np.dot(linalg.inv(S), C))
    n = np.argmax(np.abs(E))
    a = V[:,n]
    return a

def ellipse_center(a):
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    num = b*b-a*c
    x0=(c*d-b*f)/num
    y0=(a*f-b*d)/num
    return np.array([x0,y0])

def ellipse_angle_of_rotation( a ):
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    return 0.5*np.arctan(2*b/(a-c))

def ellipse_axis_length( a ):
	b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
	#TODO el abs es un machetazo para que corra.
	up = abs(2*(a*f*f+c * d*d+g*b*b-2*b*d*f-a*c*g))
	down1 = abs((b*b - a*c) * ( (c-a)*np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a)))
	down2 = abs((b*b - a*c) * ((a - c)* np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a)))
	res1=np.sqrt(up/down1)
	res2=np.sqrt(up/down2)
	return np.array([res1, res2])

def find_ellipse(x, y):
	xmean = x.mean()
	ymean = y.mean()
	x -= xmean
	y -= ymean
	a = fitEllipse(x,y)	
	center = ellipse_center(a)
	center[0] += xmean
	center[1] += ymean
	phi = ellipse_angle_of_rotation(a)
	axes = ellipse_axis_length(a)
	x += xmean
	y += ymean
	return center, phi, axes
#######Ellipse


def convertAxis(x,y):
	x2 = mx + (x - mapX1) * ax / mapLX
	y2 = my + (-y - mapY1) * ay / mapLY
	return x2,y2

def draw_robot(robotX, robotY, robotT, color=(0, 0, 170)):
	#x2 = mx + (robotX - mapX1) * ax / mapLX
	#y2 = my + (-robotY - mapY1) * ay / mapLY
	x2,y2 = convertAxis(robotX, robotY)
	rd = 8
	pygame.draw.circle(window, color, (int(x2), int(y2)), rd, 0)
	pygame.draw.line(window, (255, 248, 0), 
		(x2, y2),
		(x2 + rd * cos(robotT), y2 - rd * sin(robotT)), 2)
	

def draw_points(points, color=(255,0,0)):	
	for p in points:
		x2 ,y2= convertAxis(p.x, p.y)
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
	
	# Draw anomaly
	draw_points(particles.anomaly)
	
	######## Draw ellipse aproximation
	# Convert PointR to numpy
	points = [] 
	for p in particles.anomaly:
		points.append([p.x, p.y])	
	cnt = np.array(points, dtype=np.float32)	
		
	x = cnt[:, 0]
	y = cnt[:, 1]
	# ellipse = ((center),(width,height of bounding rect), angle)
	#center, axes, phi = cv2.fitEllipse(cnt)    
	#print "cv",(center, axes, phi)

	#pygame.draw.ellipse
	# Ellipse center
	
	#we, he = 
	#
	#pygame.draw.ellipse(window, (128,0,0), ((0,0) ,(10,20)), 1)
	
#	pygame.draw.circle(window, color, (int(x2), int(y2)), rd, 0)
	

	center, phi, axes = find_ellipse(x, y)
	print "web", (center, axes, phi)

	x1 = -center[0] - axes[0] * cos(phi)
	y1 = -center[1] - axes[0] * sin(phi)
	x2 = -center[0] + axes[0] * cos(phi)
	y2 = -center[1] + axes[0] * sin(phi)
	
	x1, y1 = convertAxis(x1,y1)
	x2, y2 = convertAxis(x2,y2)
	
	pygame.draw.line(window, (0, 180, 180), (int(x1), int(y1)), (int(x2),int(y2)))

	#
	x1 = -center[0] - axes[1] * cos(phi + pi /2 )
	y1 = -center[1] - axes[1] * sin(phi + pi / 2)
	x2 = -center[0] + axes[1] * cos(phi + pi / 2)
	y2 = -center[1] + axes[1] * sin(phi + pi / 2)
	
	x1, y1 = convertAxis(x1,y1)
	x2, y2 = convertAxis(x2,y2)
	
	pygame.draw.line(window, (0, 180, 180), (int(x1), int(y1)), (int(x2),int(y2)))
	#

	ecx, ecy = convertAxis(-1*center[0], -1* center[1])	
	pygame.draw.circle(window, (128,0,0), (int(ecx), int(ecy)) , int(10), 0)
	
	# Draw the other robots
	for orobot in particles.orobots:
		robotX, robotY, robotT = orobot.x, orobot.y, orobot.z
		draw_robot(robotX, robotY, robotT, (150,150,150))


	# Draw main robot
	robotX, robotY, robotT = particles.mrobot.x , particles.mrobot.y, particles.mrobot.z
	draw_robot(robotX, robotY, robotT)


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
