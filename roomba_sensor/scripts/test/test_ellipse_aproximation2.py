# Least squares fitting of an ellipse to point data
# using the algorithm described in:
#   Radim Halir & Jan Flusser. 1998.
#   Numerically stable direct least squares fitting of ellipses.
#   Proceedings of the 6th International Conference in Central Europe
#   on Computer Graphics and Visualization. WSCG '98, p. 125-132
#
# Adapted from the original Matlab, and  R code from Michael Bedward
# 
# David Saldana
# <dajulian at gmail.com>


# For plotting
import numpy as np
from math import *

points = [[-2, 0], [2, 0], [0.1, -0.9], [0, -1], [-2.0001, 0.001]]
cnt = np.array(points, dtype=np.float32)
#cnt = np.array(points)	

#center2, axes, phi = cv2.fitEllipse(cnt)    
#print (center2, axes, phi)

x = cnt[:, 0]
y = cnt[:, 1]
print x

d1 = np.transpose([x * x, x * y, y * y])

ones = [1] * len(x)
d2 = np.transpose([x, y, ones])

s1 = np.dot(d1.T, d1)
s2 = np.dot(d1.T, d2)
s3 = np.dot(d2.T, d2)

t = np.dot(-np.linalg.inv(s3), s2.T)

m = s1 + np.dot(s2, T)
m = np.array([m[2] / 2, -m[1], m[0] / 2])

d, evec = np.linalg.eig(m)

cond = 4 * evec[0] * evec[2] - evec[1] ** 2

a = evec[:, np.nonzero(cond > 0)]
a1 = a[:, 0][:, 0]

f = np.r_[a1, np.dot(T, a1)]



# calculate the center and lengths of the semi-axes
b2 = f[1] ** 2 / 4
center = [f[2] * f[3] / 2 - b2 * f[4],
          (f[0] * f[4] / 2 - f[1] * f[3] / 4)] / (b2 - f[0] * f[2])

num = 2 * (f[0] * f[4] ** 2 / 4 + f[2] * f[3] ** 2 / 4 + f[5] * b2 - f[1] * f[3] * f[4] / 4 - f[0] * f[2] * f[5])

den1 = b2 - f[0] * f[2]
den2 = sqrt((f[0] - f[2]) ** 2 + 4 * b2)
den3 = f[0] + f[2]

# axes
semi_axes = [sqrt(num / (den1 * (den2 - den3))), sqrt(num / (den1 * (-den2 - den3)))]

# calculate the angle of rotation
term = (f[0] - f[2]) / f[1]
angle = atan(1 / term) / 2

