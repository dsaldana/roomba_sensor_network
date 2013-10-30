#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

# For plotting
import numpy as np
from Tkinter import *


# Window size
width = 800
height = 800

# Window	
master = Tk()
wc = Canvas(master, width=width, height=height)
wc.pack()

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
	wc.create_rectangle(0, 0, width, height, fill="white")
	wc.create_line(0, 0, width, height/2)
	# Draw the particles


	#area = np.pi * ( np.array(w))
	#plt.cla()
	#plt.scatter(np.array(x), np.array(y), s=area, alpha=0.5)	
	#plt.draw()


if __name__ == '__main__':
	try:
		# Node roombaControl
		rospy.init_node('particle_drawer', anonymous=True)


		# Suscribe
		robotName = rospy.get_param('~robot_name', 'Robot1')
		rospy.Subscriber("particles", Polygon, callback)

		mainloop()


	except rospy.ROSInterruptException:
		pass
