
from geometry_msgs.msg import Point32


from roomba_sensor.map import *
from roomba_sensor.util import cut_angle


from math import *

import random
import copy

class ParticleFilter:
	# Number of particles
	N = 5000

	# Particle movement (standard deviation)
	sd_mov = 0.1

	# radio to cover
	r = 0.5

	# PF weights
	weight_tracking_left = 1.2
	weight_tracking_right = 0.01
	weight_out_of_map = 0.01
	weight_sensed_zero = 0.1

	def __init__(self):
		
		# Sparce the initial particles
		self.particles = []
		for i in range(self.N):
			p = Point32()
			# Position
			p.x = random.random() * mapLX + mapX1
			p.y = random.random() * mapLY + mapY1
			# weight
			p.z = 1.0 / self.N
			self.particles.append(p)


	def move_particles(self):		
		# Move the particles
		for p in self.particles:			
			# Normal fuction
			p.x = random.normalvariate(p.x, self.sd_mov)
			p.y = random.normalvariate(p.y, self.sd_mov)


	def resample(self):		
				
		index =  int(random.random() * self.N)
		
		# weights
		w = [0] * self.N
		for i in range(self.N):
			w[i] = self.particles[i].z

		# resampled particles
		rp = [0] * self.N

		# resample
		beta = 0.0
		mx = max (w)
		for i in range(self.N):
			beta += random.random() * 2.0 * mx
			while beta > w[index]:
				beta -= w[index]
				index = (index + 1) % self.N
			rp[i] = copy.deepcopy(self.particles[index])	

		for p in rp:
			p.z = 1.0 / self.N

		self.particles = rp
		#return rp


	# Update particle weights based on sensed values
	def update_particles(self, sensedVals):
		for [senX, senY, senT, senVal] in sensedVals:			  

			# Update particles
			for p in self.particles:
				# If the particles in the robot area.
			
				if sqrt((senX - p.x)**2 + (senY - p.y)**2) < self.r:
					if(senVal == 0):
						p.z *= self.weight_sensed_zero
					else:
						# If the anomaly was sensed
						# The robot follows the anomaly in counterclock
						# direction. Then, at left has anomaly and at
						# right has nothing.											
						left = self.evaluateSide([senX, senY], [p.x, p.y], senT)
						if left:
							p.z *= self.weight_tracking_left 
						else:
							p.z *= self.weight_tracking_right 

					
					#print "lugar errado da particula",p.z
				if p.x > mapX2 or p.x < mapX1 or p.y > mapY2 or p.y < mapY1:
					p.z = p.z * self.weight_out_of_map

	def particles_in_grid(self):
		# Particle position in grid n rows and m columns
		grid = [[0 for j in xrange(gm)] for i in xrange(gn)]
		for p in self.particles:
			# Particle in grid
			if(p.x > mapX1 and p.x < mapX2 and p.y > mapY1 and p.y < mapY2):
				i = int((p.y- mapY1) / gdy)
				j = int((p.x- mapX1) / gdx)
				#print "i,j",[i,j], ",",[p.y,p.x]
				grid[i][j] += 1
		return grid


	# Evaluate if the particle is in the right or left of
	# the robot.
	# r: robot position
	# p: particle position
	# t: robot orientation
	def evaluateSide(self, r, p, t):
		# Normal vector of robot
		#r2 = [r[0] + cos(t), r[1] + sin(t)] - r
		# Normal vector of particle
		p2 = [p[0] - r[0], p[1] - r[1]]
		#p2 /= sqrt(dotproduct(p2,p2))

		#teta = degrees(acos(-dotproduct(r2,p2)))


		pteta = atan(p2[1] / p2[0])
		if(p2[0] < 0):
			pteta += pi

		pteta = cut_angle(pteta)

		result = cut_angle(pteta - t)

		return (result>0)


