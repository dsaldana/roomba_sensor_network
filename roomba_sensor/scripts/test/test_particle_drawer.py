#!/usr/bin/env python
import rospy
from roomba_comm.msg import Particle
from roomba_comm.msg import PointR
from math import *



def run():
	rospy.init_node('test_particle_drawer')
	partPub = rospy.Publisher("/Robot1/particles", Particle)
	
	pmsg = Particle()
	pmsg.particles = []
	
	# Main robot
	robot0 = PointR()
	pmsg.mrobot = robot0
	
	# Other robots
	r1 = PointR()
	r2 = PointR()
	r3 = PointR()
	
	r1.x, r1.y = 1,2
	r2.x, r2.y = -1,-2
	r3.x, r3.y, r3.z = 0,3, pi/2
	
	pmsg.orobots = [r1, r2, r3]

	# Anomaly
	anomaly = []
	for i in range(20):
		a = PointR()
		a.x, a.y = -2+0.1*i, 2 * sin(i * 0.1)
		anomaly.append(a)
	
	pmsg.anomaly = anomaly
	
	
	while not rospy.is_shutdown():
		partPub.publish(pmsg)
		rospy.sleep(0.1)

if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass