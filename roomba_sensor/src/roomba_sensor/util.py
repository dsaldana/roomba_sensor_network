from math import *


# Restrict an angle to be only between
# -pi and pi.
def cut_angle(angle):		
	while angle > pi:
		angle -= 2 * pi
	while angle < - pi:
		angle += 2 * pi
	return angle
