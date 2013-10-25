
import copy

def sum():
	print 2+2

def resample(particles):
	print "resampling..."
	# Normalize Z
	sum = 0
	for p in particles:
		sum = sum + p.z
	print sum
	for p in particles:
		p.z = p.z / sum

	sum = 0
	for p in particles:
		sum = sum + p.z
	print "1=", sum

	
	# resampled particles
	resampledPrs = []
	for i in range(len(particles)):
		ran = random.random()
		s = 0
		for p in particles:
			s = p.z + s			
			if ran < s:
				resampledPrs.append(copy.deepcopy(p))
				break

	# Normalize resampled particles
	#print "numRes=", len(resampledPrs)
	sum = 0
	for p in resampledPrs:
		sum = sum + p.z
		#print p.z
	#print "--"
	for i in range(len(resampledPrs)):		
		resampledPrs[i].z = resampledPrs[i].z / sum
		#print p.z

	#sum = 0
	#for i in range(len(resampledPrs)):	
	#	sum = sum + resampledPrs[i].z
	#print "new1=",sum,"\n"


	return resampledPrs
