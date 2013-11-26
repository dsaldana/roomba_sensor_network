
import rospy

# Gazebo
import gazebo_msgs.srv
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage

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
			#TODO hay que revisar ese menos
			robotT =  cut_angle(-euler[0] + pi)

			return [robotX, robotY, robotT]

		except rospy.ServiceException, e:
			print "Service call to gazebo failed: %s" %e


	def getSensorPosition(self):
		[robotX, robotY, robotT] = self.getPosition()

		camX = robotX + self.d * cos(robotT)
		camY = robotY + self.d * sin(robotT)

		return [camX, camY, robotT]
		


from ar_track_alvar.msg import AlvarMarkers

class ArLocator:	
	poses = {}
	def __init__(self):
		self.load()
	
	def load(self):		
		rospy.Subscriber("/tf", TFMessage, self.callback_maker)
	
	def callback_maker(self, msg_positions):
		
		for tf in msg_positions.transforms:				
			frame_id = tf.child_frame_id.replace("ar_marker_","Robot")
			self.poses[frame_id] = tf.transform
	
	def get_robot_position(self, robotname):
		
		if robotname in self.poses:
			return self.poses[robotname]
		else:
			return None
		

	

class RealRoomba:
	# Distance from robot to camera.
	d = 0.5
	locator = None
	
	def __init__(self, robotName):
		print "Real romba localization"
		self.robotName = robotName
		self.positionServer = None
		
		if(self.locator == None):
			self.locator = ArLocator()
		#locator.add_robot(robotName)

	def get_position(self):
		pose = self.locator.get_robot_position(self.robotName)
		
		if pose == None:
			return None

		robotX, robotY = pose.translation.x, pose.translation.y
			
		# the reference for the angle is the y axes.			
		quat = [pose.rotation.w, 				
				pose.rotation.x, 
				pose.rotation.y,
				pose.rotation.z]

		euler = euler_from_quaternion(quat)
		robotT =  cut_angle(euler[0])

		return [robotX, robotY, robotT]

	def getSensorPosition(self):
		[robotX, robotY, robotT] = self.getPosition()

		camX = robotX + self.d * cos(robotT)
		camY = robotY + self.d * sin(robotT)

		return [camX, camY, robotT]

