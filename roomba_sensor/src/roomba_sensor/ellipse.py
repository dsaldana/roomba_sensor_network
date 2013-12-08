
import numpy as np
from math import *


# Least squares fitting of an ellipse to point data
# using the algorithm described in:
#   Radim Halir & Jan Flusser. 1998.
#   Numerically stable direct least squares fitting of ellipses.
#   Proceedings of the 6th International Conference in Central Europe
#   on Computer Graphics and Visualization. WSCG '98, p. 125-132
#
# Adapted from the original Matlab
#
# R Code and documentation by Michael Bedward:
# http://lastresortsoftware.blogspot.com.au/2012/09/fitting-ellipse-to-point-data.html
#
# Migrated code by David Saldana <dajulian at gmail.com>
#
# Arguments:
# x, y - the x and y coordinates of the data points
#
# Returns a list with the following elements:
#
# center - center x and y
#
# major - major semi-axis length
#
# minor - minor semi-axis length
#
# phi - rotated angle
def fit_ellipse(x, y):
	x, y = np.array(x), np.array(y)
	
	d1 = np.transpose([x*x, x*y, y*y])

	ones = [1] * len(x)
	d2 = np.transpose([x, y, ones])


	s1 = np.dot(d1.T,d1)
	s2 = np.dot(d1.T, d2)
	s3 = np.dot(d2.T, d2)



	t = np.dot(-np.linalg.inv(s3), s2.T)

	m =  s1 + np.dot(s2,t)
	m = np.array([m[2] / 2, -m[1], m[0]/2])

	d, evec = np.linalg.eig(m)

	cond = 4 * evec[0] * evec[2] - evec[1]**2

	a = evec[:, np.nonzero(cond>0)]
	a1 = a[:,0][:,0]

	f = np.r_[a1, np.dot(t,a1)]



	# calculate the center and lengths of the semi-axes
	b2 = f[1] ** 2 / 4 
	center = [f[2] * f[3] / 2 - b2 * f[4], 
		(f[0] * f[4] / 2 - f[1] * f[3] / 4)] / (b2 - f[0] * f[2])


	num = 2 * (f[0] * f[4]**2 / 4 + f[2] * f[3]**2 / 4 + f[5] * b2 - f[1]*f[3]*f[4]/4 - f[0]*f[2]*f[5])

	den1 = b2 - f[0]*f[2]
	den2 = sqrt((f[0] - f[2])**2 + 4*b2)
	den3 = f[0] + f[2]

	# axes
	semi_axes = [ sqrt(num / (den1 * (den2 - den3))), sqrt(num / (den1 * (-den2 - den3)))]

	# calculate the angle of rotation
	term = (f[0] - f[2]) / f[1]
	phi = atan(1 / term) / 2

	return center, semi_axes, phi

#
# Fit the ellipse in a box
#
# Return 2 points that defines a box
#
def ellipse_box(center, axes, phi):
	p1x = center[0] + axes[0] * cos(phi) + center[0] + axes[1] * cos(phi + pi / 2)
	p1y = center[1] + axes[0] * sin(phi) + center[1] + axes[1] * sin(phi + pi / 2)
	
	p2x = center[0] - axes[0] * cos(phi) - center[0] - axes[1] * cos(phi + pi / 2)
	p2y = center[1] - axes[0] * sin(phi) - center[1] - axes[1] * sin(phi + pi / 2)

	return p1x, p1y, p2x, p2y

#
# Calculate points on an ellipse.
#
# Return a set of points gerated from the ellipse.
#
def ellipse_points(center, axes, phi, n=500):	
	# Calculate points on an ellipse described by
	# the fit argument as returned by fit.ellipse
	#
	# n is the number of points to render
	tt = np.linspace(0, 2 * pi, n, endpoint=True, retstep=False)
	print degrees(phi)
	sa = sin(phi)
	ca = cos(phi)
	ct = np.cos(tt)
	st = np.sin(tt)

	max = np.min(axes)
	min = np.max(axes)
	
	# X,Y points
	x = center[0] + max * ct * ca - min * st * sa
	y = center[1] + max * ct * sa + min * st * ca

	return x, y





