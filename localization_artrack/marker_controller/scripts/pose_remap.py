#!/usr/bin/env python
import roslib
roslib.load_manifest('marker_controller')

import rospy

from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from marker_controller.msg import TargetPoses
from marker_controller.msg import TargetPose

def marker_callback(marker_info):
	global base_publisher
	global target_publisher
	global deposit_publisher
	global robot_id
	global deposit_id

	if not ( base_publisher or target_publisher or robot_id ): return

	targets = TargetPoses()

	for marker in marker_info.markers:
		if marker.id == robot_id:
			base_publisher.publish( marker.pose )
		elif marker.id == deposit_id:
			deposit_publisher.publish( marker.pose )
		else:
			tg = TargetPose()
			tg.id = marker.id
			tg.pose = marker.pose

			targets.targets.append( tg )

	target_publisher.publish( targets )

if __name__ == '__main__':
	global base_publisher
	global target_publisher
	global deposit_publisher
	global robot_id
	global deposit_id

	base_publisher = None

	# Init note
	rospy.init_node('pose_remap')

	# Get robot name
	robot_name = rospy.get_param('~robot_name', 'mobile_base')
	robot_id = rospy.get_param("~robot_id", 0)
	deposit_id = rospy.get_param("~deposit_id", 4)

	# Subscribe to pose marker topic
	rospy.Subscriber("/ar_pose_marker", AlvarMarkers, marker_callback)

	# The redirect topic
	base_publisher = rospy.Publisher("/%s/pose" % robot_name, PoseStamped)
	target_publisher = rospy.Publisher("/%s/targets" % robot_name, TargetPoses)
	deposit_publisher = rospy.Publisher("/%s/deposit" % robot_name, PoseStamped)

	# Wait
	rospy.spin()
