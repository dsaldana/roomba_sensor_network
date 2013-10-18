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

	# Control the car.
	topicName = "/" + robotName + "/commands/velocity"
	velPub = rospy.Publisher(topicName, Twist)


	while not rospy.is_shutdown():
		vel = Twist()
		vel.linear.x = 1.10
		print("publicando publicando mic mic.")
		velPub.publish(vel)

		rospy.sleep(0.10)


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
