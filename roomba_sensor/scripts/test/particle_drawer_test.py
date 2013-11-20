#!/usr/bin/env python
import rospy
from roomba_comm.msg import SensedValue




def run():
	while not rospy.is_shutdown():
		rospy.sleep(0.1)

if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass