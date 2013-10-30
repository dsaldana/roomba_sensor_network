#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

# For plotting
import numpy as np
import sys
import pygame
import time


# Window size
width = 800
height = 600

# Window	
pygame.init() 
window = pygame.display.set_mode((width, height)) 

# From roomba_cotrol
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


def callback(particles):
	N = len(particles.points)
	x = [0] * N
	y = [0] * N
	w = [0] * N

	for i in xrange(N):
		x[i] = particles.points[i].x
		y[i] = particles.points[i].y
		w[i] = particles.points[i].z
		
	# Draw the canvas
	window.fill((255, 255, 255))

	# margin
	mx = width / 10.0
	my = height / 10.0
	# Draw the grid
	ax = width - 2.0 * mx # area for x
	ay = height - 2.0 * my # area for y
	dx = (ax) / gm #Delta x
	dy = (ay) / gn #Delta y


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
	pygame.draw.line(window, (0, 0, 250), (mx/2, my+dy*gn/2), (width-mx/2, my+dy*gn/2))
	pygame.draw.line(window, (0, 0, 250), (mx+dx*gm/2, my/2), (mx+dx*gm/2, height-my/2))
		
	# Draw the particles
	pcolor = (255,0,0)
	for i in range(N):
		x2 = mx + (x[i] - mapX1) * ax / mapLX
		y2 = my + (-y[i] - mapY1) * ay / mapLY
		pygame.draw.circle(window, pcolor, (int(x2),int(y2)), 2, 0)


	#area = np.pi * ( np.array(w))
	#plt.cla()
	#plt.scatter(np.array(x), np.array(y), s=area, alpha=0.5)	
	#plt.draw()
	
	pygame.display.flip() 

if __name__ == '__main__':
	try:
		# Node roombaControl
		rospy.init_node('particle_drawer', anonymous=True)		

		# Suscribe
		robotName = rospy.get_param('~robot_name', 'Robot1')
		rospy.Subscriber("particles", Polygon, callback)
		
		# Window
		while True: 
			time.sleep(0.1)
			for event in pygame.event.get(): 
				if event.type == pygame.QUIT: 
					sys.exit(0) 
			  #else: 
				  #print event 

	except rospy.ROSInterruptException:
		pass
