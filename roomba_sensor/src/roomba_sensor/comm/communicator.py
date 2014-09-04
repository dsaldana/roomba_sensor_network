from geometry_msgs.msg import Point32
import rospy

from roomba_comm.msg import Particle
from roomba_comm.msg import PointR
from roomba_comm.msg import Point
from roomba_comm.msg import SensedValue


class Communicator(object):
    """
    Publish and consume messages to ROS topics.
    """

    def __init__(self, robot_name):
        self.robot_name = robot_name
        # Last message for each robot. {robot: message}.
        self.robot_msgs = {}

        # Robot communication
        # Subscriber for robot communication
        rospy.Subscriber("/robotCom", SensedValue, self.robot_comm)
        # Sensor's publisher
        self.sensor_pub = rospy.Publisher("/robotCom", SensedValue, queue_size=1)
        # Create a publisher for the particles
        self.particle_pub = rospy.Publisher("/" + robot_name + "/particles", Particle, queue_size=1)

        # Create the Publisher to control the robot.
        # topic_name = "/" + robot_name + "/commands/velocity"
        # vel_pub = rospy.Publisher(topic_name, Twist)

        # Goal Navigator
        self.nav_pub = rospy.Publisher("/" + robot_name + "/goal", Point32, queue_size=1)
        # Tracker navigator
        self.track_pub = rospy.Publisher("/" + robot_name + "/tracking", Point32, queue_size=1)

    def robot_comm(self, msg):
        """
        Callback for robot communication.
        Receive messages from all robots (even the robot which is running this script).
        :param msg:
        """
        self.robot_msgs[msg.robot_id] = msg

    def read_inbox(self):
        """
        Read latest message from each robot.
        :param am: anomaly manager
        :return:
            robot_positions: position for each robot.
            sensed_points: position and value for each sensed point {id_robot: [x, y, th, value]}
            polygons: detected anomalies {id_robot:[polygon, closed, time]}
        """
        # This variable contain all robots (even this robot)
        robot_positions = []
        # Sensed values
        sensed_points = {}

        # Anomaly polygons {id_robot:[polygon, closed, time]}
        polygons = {}
        for msg in self.robot_msgs.values():
            # todo validate data do msg. old messages must be deleted.
            sensed_points[msg.robot_id] = [msg.x, msg.y, msg.theta, msg.value]
            # Robot position
            rp = PointR()
            rp.x, rp.y, rp.z = msg.rx, msg.ry, msg.rtheta
            rp.robot_id = msg.robot_id
            robot_positions.append(rp)

            # Polygon
            if msg.anomaly:
                polygon = [(p.x, p.y) for p in msg.anomaly]
                polygons[msg.robot_id] = [polygon, msg.closed_anomaly, msg.time_of_detection]

        return robot_positions, sensed_points, polygons

    def send_sensed_value(self, sensed_val, camera_position, robot_position,
                          polygon=None, closed_anomaly=False, time_of_detection=0):
        """

        :param sensed_val:
        :param camera_position:
        :param robot_position:
        :param polygon: list of tuples (x,y)
        :param closed_anomaly:
        :param time_of_detection:
        """
        smsg = SensedValue()
        smsg.x, smsg.y, smsg.theta = camera_position
        smsg.rx, smsg.ry, smsg.rtheta = robot_position
        smsg.robot_id = self.robot_name
        smsg.value = sensed_val

        # for anomaly in polygon
        if polygon is not None:
            smsg.anomaly = [Point(i[0], i[1]) for i in polygon]
            smsg.closed_anomaly = closed_anomaly
            # smsg.time_of_detection = time_of_detection
        # Publish
        self.sensor_pub.publish(smsg)

    def publish_particles(self, particles, robot_position, orobots, polyline):
        #
        anomaly_points = self._polyline_to_anomaly_points(polyline)

        msg_parts = Particle()
        msg_parts.particles = particles
        mrobot = PointR()
        mrobot.x, mrobot.y, mrobot.z = robot_position
        msg_parts.mrobot = mrobot
        msg_parts.orobots = orobots
        msg_parts.anomaly = anomaly_points

        self.particle_pub.publish(msg_parts)

    @staticmethod
    def _polyline_to_anomaly_points(polyline):
        """
        Convert polyline to anomaly points(class PointR).

        :return: vector of PointR
        """
        anomaly_points = []
        for p in polyline:
            an = PointR()
            an.x, an.y = p
            anomaly_points.append(an)
        return anomaly_points

    def publish_goal(self, goal):
        p = Point32()
        p.x, p.y = goal
        self.nav_pub.publish(p)

    def publish_track(self, control_p, crf):
        p = Point32()
        p.x = control_p
        p.y = crf
        # print "f=", crf
        self.track_pub.publish(p)