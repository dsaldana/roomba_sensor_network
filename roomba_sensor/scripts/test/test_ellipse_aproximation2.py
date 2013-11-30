# Least squares fitting of an ellipse to point data
# using the algorithm described in:
#   Radim Halir & Jan Flusser. 1998.
#   Numerically stable direct least squares fitting of ellipses.
#   Proceedings of the 6th International Conference in Central Europe
#   on Computer Graphics and Visualization. WSCG '98, p. 125-132
#
# Adapted from the original Matlab code by David Saldana
# <dajulian at gmail.com>


# For plotting
import numpy as np
from math import *

points = [[-2,0], [2,0], [0.1, -0.9], [0, -1], [-2.0001,0.001]]
cnt = np.array(points, dtype=np.float32)	
#cnt = np.array(points)	

#center2, axes, phi = cv2.fitEllipse(cnt)    
#print (center2, axes, phi)

x = cnt[:, 0]
y = cnt[:, 1]
print x

# TODO ???
eps = 1.0e-8

##D1 <- cbind(dat$x * dat$x, dat$x * dat$y, dat$y * dat$y)
d1 = np.transpose([x*x, x*y, y*y])

##D2 <- cbind(dat$x, dat$y, 1)
ones = [1] * len(x)
d2 = np.transpose([x, y, ones])

##S1 <- t(D1) %*% D1
s1 = np.dot(d1.T,d1)
##S2 <- t(D1) %*% D2
s2 = np.dot(d1.T, d2)
##S3 <- t(D2) %*% D2
s3 = np.dot(d2.T, d2)


##T <- -solve(S3) %*% t(S2)
t = np.dot(-np.linalg.inv(s3), s2.T)
##M <- S1 + S2 %*% T
m =  s1 + np.dot(s2,T)
##M <- rbind(M[3,] / 2, -M[2,], M[1,] / 2)
m = np.array([m[2] / 2, -m[1], m[0]/2])
##evec <- eigen(M)$vec
d, evec = np.linalg.eig(m)
##cond <- 4 * evec[1,] * evec[3,] - evec[2,]^2
cond = 4 * evec[0] * evec[2] - evec[1]**2
##a1 <- evec[, which(cond > 0)]
a = evec[:, np.nonzero(cond>0)]
a1 = a[:,0][:,0]
##f <- c(a1, T %*% a1)
f = np.dot(T,a1)
##names(f) <- letters[1:6]



M = np.array([[0.009663637,  0.002229428,  0.002231076],
[-0.019324897, -0.004460061, -0.004458856],
[0.041872617,  0.009662448,  0.009663637]])