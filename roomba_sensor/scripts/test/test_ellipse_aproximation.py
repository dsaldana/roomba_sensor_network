#!/usr/bin/env python

import cv2

# For plotting
import numpy as np
from math import *


###### ellipse
## FROM http://stackoverflow.com/questions/13635528/fit-a-ellipse-in-python-given-a-set-of-points-xi-xi-yi
import numpy.linalg as linalg
def fitEllipse(x,y):
    x = x[:,np.newaxis]
    y = y[:,np.newaxis]
    D =  np.hstack((x*x, x*y, y*y, x, y, np.ones_like(x)))
    S = np.dot(D.T,D)
    C = np.zeros([6,6])
    C[0,2] = C[2,0] = 2; C[1,1] = -1
    E, V =  linalg.eig(np.dot(linalg.inv(S), C))
    n = np.argmax(np.abs(E))
    a = V[:,n]
    return a

def ellipse_center(a):
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    num = b*b-a*c
    x0=(c*d-b*f)/num
    y0=(a*f-b*d)/num
    return np.array([x0,y0])

def ellipse_angle_of_rotation( a ):
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    return 0.5*np.arctan(2*b/(a-c))

def ellipse_axis_length( a ):
	b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
	#TODO el abs es un machetazo para que corra.
	up = abs(2*(a*f*f+c * d*d+g*b*b-2*b*d*f-a*c*g))
	down1 = abs((b*b - a*c) * ( (c-a)*np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a)))
	down2 = abs((b*b - a*c) * ((a - c)* np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a)))
	print up,down2
	res1=np.sqrt(up / down1)
	res2=np.sqrt(up / down2)
	return np.array([res1, res2])

def find_ellipse(x, y):
	xmean = x.mean()
	ymean = y.mean()
	x -= xmean
	y -= ymean
	a = fitEllipse(x,y)

	center = ellipse_center(a)
	center[0] += xmean
	center[1] += ymean
	phi = ellipse_angle_of_rotation(a)
	axes = ellipse_axis_length(a)
	x += xmean
	y += ymean
	return center, phi, axes
#######Ellipse


points = [[-2.0, 0.0], [-1.7999999523162842, 0.199666827917099], [-1.600000023841858, 0.3973386585712433], [-1.399999976158142, 0.5910404324531555], [-1.2000000476837158, 0.7788366675376892], [-1.0, 0.9588510990142822], [-0.800000011920929, 1.1292849779129028], [-0.6000000238418579, 1.2884353399276733], [-0.4000000059604645, 1.4347121715545654], [-0.20000000298023224, 1.566653847694397], [0.0, 1.6829419136047363], [0.20000000298023224, 1.7824146747589111], [0.4000000059604645, 1.864078164100647], [0.6000000238418579, 1.9271163940429688], [0.800000011920929, 1.9708994626998901], [1.0, 1.9949899911880493], [1.2000000476837158, 1.9991471767425537], [1.399999976158142, 1.9833296537399292], [1.600000023841858, 1.947695255279541], [1.7999999523162842, 1.892600178718567]]
points = [[-2,0], [2,0], [0.5, 0.5], [0.5, -0.1], [-1,0.8]]
points = [[-2,0], [2,0], [0, 1], [0, -1], [-2.0001,0.0001]]
cnt = np.array(points, dtype=np.float32)	

#center2, axes, phi = cv2.fitEllipse(cnt)    
#print (center2, axes, phi)

x = cnt[:, 0]
y = cnt[:, 1]
print find_ellipse(x, y)





