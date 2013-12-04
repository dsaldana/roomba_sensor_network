
import rospy

# Gazebo
import gazebo_msgs.srv
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage

from math import *
from roomba_sensor.util import cut_angle
from roomba_sensor.particle_filter import ParticleFilter

#######################################################
##### Localization for virtual or physical robot.
#######################################################
class RoombaLocalization:

	def __init__(self, robotName):
		simulated_robots = rospy.get_param('/simulated_robots', False)
		# Object to get information from Gazebo		
		self.robot = None

		if simulated_robots:
			rospy.loginfo("Working with simulated robots.")
			# Localization by Gazebo
			self.robot = RoombaGazebo(robotName)
		else:
			rospy.loginfo("Working with physical robots.")
			# Localization by ar_alvar
			self.robot = RealRoomba(robotName)

	def get_position(self):
		return self.robot.get_position()

	def get_sensor_position(self):
		return self.robot.get_sensor_position()




######################################
### Localization from Gazebo.
######################################
class RoombaGazebo:
	# Distance from robot to camera.
	d = 0.5
	


	def __init__(self, robotName):
		print "RoombaGazebo"
		self.robotName = robotName
		self.positionServer = None
		self.load()


	def load(self):		
		rospy.logerr("waiting model state service from Gazebo")
		rospy.wait_for_service('/gazebo/get_model_state')		
		self.positionServer = rospy.ServiceProxy('/gazebo/get_model_state', 
			gazebo_msgs.srv.GetModelState)


	def get_position(self):
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
			# That minus is a little bit strage but it works well.
			robotT =  cut_angle(-euler[0] + pi)

			return [robotX, robotY, robotT]

		except rospy.ServiceException, e:
			print "Service call to gazebo failed: %s" %e


	def get_sensor_position(self):
		[robotX, robotY, robotT] = self.get_position()

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
		

######################################################################
# This class gets the robot position based on ar_track_alvar package.	
######################################################################
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
			rospy.logerr("No robot position")
			return [0, 0, 0]

		robotX, robotY = pose.translation.x, pose.translation.y
			
		# the reference for the angle is the y axes.			
		quat = [pose.rotation.w, 				
				pose.rotation.x, 
				pose.rotation.y,
				pose.rotation.z]

		euler = euler_from_quaternion(quat)
		robotT =  cut_angle(euler[0])

		return [robotX, robotY, robotT]

	def get_sensor_position(self):
		[robotX, robotY, robotT] = self.get_position()

		camX = robotX + self.d * cos(robotT)
		camY = robotY + self.d * sin(robotT)

		return [camX, camY, robotT]