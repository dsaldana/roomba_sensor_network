#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def run():
	# Node roombaControl
	rospy.init_node('roomba_control')

	# Robot's name is an argument
	robotName = rospy.get_param('~robot_name', 'Robot1')
	print robotName

	# Create the publisher to control the robot.
	topicName = "/" + robotName + "/commands/velocity"
	velPub = rospy.Publisher(topicName, Twist)

	# Wait while the world is totally spawned.
	rospy.sleep(5.0)

	print "start"
	while not rospy.is_shutdown():
		vel = Twist()
		vel.linear.x = 1.00
		vel.angular.z = -1.50		
		#velPub.publish(vel)

		rospy.sleep(0.50)


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
