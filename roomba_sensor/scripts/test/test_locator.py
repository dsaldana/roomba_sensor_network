#!/usr/bin/env python
import rospy
from ar_track_alvar.msg import AlvarMarkers
from math import *

from roomba_sensor.roomba import RealRoomba

def run():
	rospy.init_node('test_locator')
	
	locator = RealRoomba("Robot1")
	
	
	while not rospy.is_shutdown():
		print locator.get_position()
		rospy.sleep(0.1)

if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass