#!/usr/bin/env python
from geometry_msgs.msg import Point32
import random
import rospy

execfile('resampler.py')

print "Start!"

# number of particles
N = 100
# Sparce the initial particles
particles = []
for i in range(N):
	p = Point32()
	# Position
	p.x = random.random() 
	p.y = random.random() 
	# weight
	p.z = 1.0 / N
	particles.append(p)

while True:
	# change weights
	for p in particles:
		if random.random() < 0.3:
			p.z = (p.z) * (0.1)

	particles = resample(particles)

	rospy.sleep(2.50)
