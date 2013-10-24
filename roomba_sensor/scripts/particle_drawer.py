#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32


def callback(particles):
	for p in particles.points:
		print p




if __name__ == '__main__':
	try:
		# Node roombaControl
		rospy.init_node('particle_drawer', anonymous=True)
		robotName = rospy.get_param('~robot_name', 'Robot1')
		rospy.Subscriber(robotName + "/particles", Polygon, callback)
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
