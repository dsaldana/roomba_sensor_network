import rospy

# Gazebo
import gazebo_msgs.srv
from tf.transformations import euler_from_quaternion
#from tf2_msgs.msg import TFMessage


from math import *
from roomba_sensor.util.angle import cut_angle



# Distance from robot to camera.
d = rospy.get_param('/sensor_distance', 0.5)


class RoombaLocalization(object):
    """
    Localization for virtual or physical robot.
    :param robot_name:
    """
    def __init__(self, robot_name):
        simulated_robots = rospy.get_param('/simulated_robots', False)
        # Object to get information from Gazebo
        self.robot = None

        if simulated_robots:
            rospy.loginfo("Working with simulated robots.")
            # Localization by Gazebo
            self.robot = RoombaGazebo(robot_name)
        else:
            rospy.loginfo("Working with physical robots.")
            # Localization by ar_alvar
            self.robot = RealRoomba(robot_name)

    def get_position(self):
        return self.robot.get_position()

    def get_sensor_position(self):
        return self.robot.get_sensor_position()


######################################
class RoombaGazebo(object):
    """
    Localization from Gazebo.
    :param robotName:
    """

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
            robotT = cut_angle(- euler[0] + pi)

            return [robotX, robotY, robotT]

        except rospy.ServiceException, e:
            print "Service call to gazebo failed: %s" % e


    def get_sensor_position(self):
        [robotX, robotY, robotT] = self.get_position()

        camX = robotX + d * cos(robotT)
        camY = robotY + d * sin(robotT)

        return [camX, camY, robotT]

########################################################

class ArLocator(object):
    """
    Localization for physical robots, using ARTrack Alvar.
    """
    poses = {}
    def __init__(self):
        self.load()

    def load(self):
        from ar_track_alvar_msgs.msg import AlvarMarkers
        # rospy.Subscriber("/tf", TFMessage, self.callback_maker)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback_maker)

    def callback_maker(self, msg_positions):

        for marker in msg_positions.markers:
            detected_robot = "Robot%i" % marker.id
            self.poses[detected_robot] = marker.pose.pose


    def get_robot_position(self, robotname):
        if robotname in self.poses:
            return self.poses[robotname]
        else:
            return None


######################################################################
# This class gets the robot position based on ar_track_alvar package.	
######################################################################
class RealRoomba:
    locator = None

    def __init__(self, robotName):
        print "Real romba localization"
        self.robotName = robotName
        self.positionServer = None

        if self.locator == None:
            self.locator = ArLocator()
        #locator.add_robot(robotName)

    def get_position(self):
        pose = self.locator.get_robot_position(self.robotName)

        if pose is None:
            rospy.logerr("No robot position")
            return [0, 0, 0]

        robotX, robotY = pose.position.x, pose.position.y

        # the reference for the angle is the y axes.
        quat = [pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z]

        euler = euler_from_quaternion(quat)

        ## FIXME It works, I dont know why.
        robotT = cut_angle(-euler[0] + pi)

        return [robotX, robotY, robotT]

    def get_sensor_position(self):
        [robotX, robotY, robotT] = self.get_position()

        cam_x = robotX + d * cos(robotT)
        cam_y = robotY + d * sin(robotT)

        return [cam_x, cam_y, robotT]
