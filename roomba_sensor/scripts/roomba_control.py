#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def run():
	# Node roombaControl
    rospy.init_node('roomba_control')


	# Control the car.
    velPub = rospy.Publisher("/robot1/cmd_vel_mux/input/teleop", Twist)


    while not rospy.is_shutdown():
		vel = Twist()
		vel.linear.x = 1.0
		print("publicando publicando mic mic.")
		velPub.publish(vel)

		rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
