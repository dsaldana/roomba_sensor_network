
import rospy

# Gazebo
import gazebo_msgs.srv
from tf.transformations import euler_from_quaternion

from math import *
from roomba_sensor.util import cut_angle
from roomba_sensor.particle_filter import ParticleFilter

class RoombaGazebo:
	# Distance from robot to camera.
	d = 0.5
	


	def __init__(self, robotName):
		print "RoombaGazebo"
		self.robotName = robotName
		self.positionServer = None
		self.load()


	def load(self):
		print "wait for service"
		rospy.wait_for_service('/gazebo/get_model_state')
		self.positionServer = rospy.ServiceProxy('/gazebo/get_model_state', gazebo_msgs.srv.GetModelState)


	def getPosition(self):
		try:			
			resp = self.positionServer(self.robotName, "world")
			robotX = resp.pose.position.x
			robotY = resp.pose.position.y
			# the reference for the angle is the y axes.			
			quat = [resp.pose.orientation.w, 
				resp.pose.orientation.x,
				resp.pose.orientation.y, 
				resp.pose.orientation.z]

			euler = euler_from_quaternion(quat)
			robotT =  cut_angle(-euler[0] + pi)

			return [robotX, robotY, robotT]

		except rospy.ServiceException, e:
			print "Service call to gazebo failed: %s" %e


	def getSensorPosition(self):
		[robotX, robotY, robotT] = self.getPosition()

		camX = robotX + self.d * cos(robotT)
		camY = robotY + self.d * sin(robotT)

		return [camX, camY, robotT]
		



