#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

# For plotting
import numpy as np
import matplotlib.pyplot as plt

hl, = plt.plot([], [])

def callback(particles):
	x = []
	y = []
	w = []

	for p in particles.points:
		x.append(p.x)
		y.append(p.y)
		w.append(p.z)
		

	# Draw the particles
	area = np.pi * (200 * np.array(w))
	plt.cla()
	plt.scatter(np.array(x), np.array(y), s=area, alpha=0.5)	
	plt.draw()


if __name__ == '__main__':
	try:
		# Node roombaControl
		rospy.init_node('particle_drawer', anonymous=True)
		robotName = rospy.get_param('~robot_name', 'Robot1')
		rospy.Subscriber(robotName + "/particles", Polygon, callback)

		plt.show()


	except rospy.ROSInterruptException:
		pass
