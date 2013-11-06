
from geometry_msgs.msg import Point32


from roomba_sensor.map import *
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
		print "resampling..."
				
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
			p.z /= mx

		self.particles = rp
		#return rp


	# Update particle weights based on sensed values
	def update_particles(self, sensedVals):
		for [senX, senY, senVal] in sensedVals:			  

			# Update particles
			for p in self.particles:
				# If the particles in the robot area.
			
				if sqrt((senX - p.x)**2 + (senY - p.y)**2) < self.r:
					if(senVal == 0):
						p.z *= 0.1
					else:
						# If the anomaly was sensed
						p.z *= 1.1 * senVal 
					
					#print "lugar errado da particula",p.z
				if p.x > mapX2 or p.x < mapX1 or p.y > mapY2 or p.y < mapY1:
					p.z = p.z * 0.1

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