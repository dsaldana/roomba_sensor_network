
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32

from roomba_comm.msg import Particle
from roomba_comm.msg import PointR
from roomba_comm.msg import SensedValue

import rospy


class Communicator(object):
    def __init__(self, robot_name):
        self.robot_name = robot_name
        # Last message for each robot. {robot: message}.
        self.robot_msgs = {}

        # Robot communication
        # Subscriber for robot communication
        rospy.Subscriber("/robotCom", SensedValue, self.robot_comm)
        # Sensor's publisher
        self.sensor_pub = rospy.Publisher("/robotCom", SensedValue)
        # Create a publisher for the particles
        self.particle_pub = rospy.Publisher("/" + robot_name + "/particles", Particle)

        # Create the Publisher to control the robot.
        # topic_name = "/" + robot_name + "/commands/velocity"
        # vel_pub = rospy.Publisher(topic_name, Twist)

        # Goal Navigator
        self.nav_pub = rospy.Publisher("/" + robot_name + "/goal", Point32)
        # Tracker navigator
        self.track_pub = rospy.Publisher("/" + robot_name + "/tracking", Point32)

    def robot_comm(self, msg):
        """
        Callback for robot communication.
        Receive messages from all robots (even the robot which is running this script).
        :param msg:
        """
        self.robot_msgs[msg.robot_id] = msg

    def read_inbox(self, am):
        """
        Read last message from each robot.
        :param am: anomaly manager
        :return:
        """
        # This variable contain all robots (even this robot)
        orobots = []
        # Sensed values
        samples = []

        for msg in self.robot_msgs.values():
            samples.append([msg.x, msg.y, msg.theta, msg.value])
            # Other robot positions
            orobot = PointR()
            orobot.x, orobot.y, orobot.z = msg.rx, msg.ry, msg.rtheta
            orobot.robot_id = msg.robot_id
            orobots.append(orobot)

            if msg.value > 0:
                # new point with anomaly
                an = PointR()
                an.x, an.y = msg.rx, msg.ry
                an.robot_id = msg.robot_id

                ### Anomaly polygon
                am.add_anomaly_point(msg.robot_id, (msg.rx, msg.ry))

        # All messages are read
        self.robot_msgs.clear()

        return orobots, samples

    def send_sensed_value(self, sensed_val, camera_position, robot_position):
        smsg = SensedValue()
        smsg.x, smsg.y, smsg.theta = camera_position
        smsg.rx, smsg.ry, smsg.rtheta = robot_position

        smsg.robot_id = self.robot_name
        smsg.value = sensed_val
        self.sensor_pub.publish(smsg)

    def publish_particles(self, particles, robot_position, orobots, anomaly_points):
        msg_parts = Particle()
        msg_parts.particles = particles
        mrobot = PointR()
        mrobot.x, mrobot.y, mrobot.z = robot_position
        msg_parts.mrobot = mrobot
        msg_parts.orobots = orobots
        # msg_parts.anomaly = anomaly_points
        msg_parts.anomaly = anomaly_points
        self.particle_pub.publish(msg_parts)

    def publish_goal(self, goal):
        p = Point32()
        p.x, p.y = goal
        self.nav_pub.publish(p)

    def publish_track(self,control_p, crf):
        p = Point32()
        p.x = control_p
        p.y = crf
        #print "f=", crf
        self.track_pub.publish(p)