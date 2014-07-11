#!/usr/bin/env python
# Image
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32

from roomba_comm.msg import Particle
from roomba_comm.msg import PointR
from roomba_comm.msg import SensedValue


# OpenCV
from cv_bridge import CvBridge
import cv

# Math
from math import *

from roomba_sensor.roomba import RoombaLocalization
from roomba_sensor.particle_filter import ParticleFilter
from roomba_sensor.grid_util import coords_to_grid

# Map configuration
from roomba_sensor.map import *


# Numpy
import numpy as np

# Clustering
from scipy.cluster.vq import kmeans2


# Sensed value by the camera
sensedValue = 0
# How many white pixels in the left
sensedLeft = 0
# How many white pixels in the right
sensedRight = 0

robotName = "r0"

robot_msgs = {}

# Callback for robot communication.
def robot_comm(msg):
    # Message from the same robot
    #if (msg.robot_id == robotName):
    #	return
    # Update a list of values
    robot_msgs[msg.robot_id] = msg


# Callback for camera sensor.
def img_callback(img):
    global sensedValue
    global sensedLeft
    global sensedRight

    sensor_enabled = rospy.get_param('/sensor_enabled', True)

    if not sensor_enabled:
        return

    #OpenCV matrix
    mat = CvBridge().imgmsg_to_cv(img, "rgb8")
    # Extract red channel
    red_channel = cv.CreateImage(cv.GetSize(mat), 8, 1)
    green_channel = cv.CreateImage(cv.GetSize(mat), 8, 1)
    blue_channel = cv.CreateImage(cv.GetSize(mat), 8, 1)
    cv.Split(mat, red_channel, green_channel, blue_channel, None)


    # How many white pixels in the left
    pl = 0
    # How many white pixels in the right
    pr = 0

    threshold_value = 160
    threshold_other = 160

    # Conunt the pixels in the middel row of the image.
    for i in [int(mat.rows / 2)]:
        for j in xrange(mat.cols / 2):
            if red_channel[i, j] > threshold_value and green_channel[i, j] < threshold_other and blue_channel[
                i, j] < threshold_other:
                pl = j

        for j in xrange(int(mat.cols / 2) + 1, mat.cols):
            if red_channel[i, j] > threshold_value and green_channel[i, j] < threshold_other and blue_channel[
                i, j] < threshold_other:
                pr = j - int(mat.cols / 2)

    total = mat.rows * mat.cols * 1.0
    sensedValue = (pl + pr) / total
    sensedLeft = (pl * 2.0) / (mat.rows)
    sensedRight = (pr * 2.0) / (mat.rows)


#print [total, sensedValue, sensedLeft, sensedRight]

# Convert two points to a verctor. 
# Return magnitude and angle.
def points_to_vector(p1, p2):
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]

    # Magnitude. Coulomb law.
    mag = sqrt(dx ** 2 + dy ** 2)

    # Angle
    theta = atan(dy / dx)

    if dx < 0:
        theta += pi

    return mag, theta


# Return x,y components.
def vector_components(mag, theta):
    # components
    x, y = mag * cos(theta), mag * sin(theta)
    return x, y


def run():
    ######### Initialization ##################
    # Node roombaControl
    rospy.init_node('roomba_control')

    # Robot's name is an argument
    global robotName
    robotName = rospy.get_param('~robot_name', 'Robot1')

    # Constant in columbs law. Force by centroid.
    f_centroid = rospy.get_param('/f_centroid', 1.0)
    f_robots = rospy.get_param('/f_robots', 2.0)

    rospy.loginfo("Loading robot control.")

    # Create the Publisher to control the robot.
    topicName = "/" + robotName + "/commands/velocity"
    velPub = rospy.Publisher(topicName, Twist)


    # Create a publisher for the particles
    partPub = rospy.Publisher("/" + robotName + "/particles", Particle)

    # Camera
    topicName = "/" + robotName + "/front_cam/camera/image"
    image_sub = rospy.Subscriber(topicName, Image, img_callback, queue_size=1)

    # Robot communication
    # Subscriber for robot communication
    rospy.Subscriber("/robotCom", SensedValue, robot_comm)
    # Sensor's publisher
    sensorPub = rospy.Publisher("/robotCom", SensedValue)

    # Goal Navigator
    navPub = rospy.Publisher("/" + robotName + "/goal", Point32)
    # Tracker navigator
    trackPub = rospy.Publisher("/" + robotName + "/tracking", Point32)

    # Initialize Particles
    pf = ParticleFilter()

    ## Robot Localization
    robot = RoombaLocalization(robotName)

    # Explore flag. False for tracking
    explore = True

    # Time for tracking without sensing anomaly.
    max_tracking_time = 5
    # last time that an anomaly was detected
    last_time_anomaly = 0

    anomaly_points = []

    cents = None

    k_skip = -1

    ######## Control Loop
    print "Start!"
    while not rospy.is_shutdown():
        # Get robot position from gazebo
        [robotX, robotY, robotT] = robot.get_position()
        # Camera position
        [camX, camY, camT] = robot.get_sensor_position()

        # Send the info to other robots.
        smsg = SensedValue()
        smsg.x, smsg.y, smsg.theta = camX, camY, camT
        smsg.rx, smsg.ry, smsg.rtheta = robotX, robotY, robotT

        smsg.robot_id = robotName
        smsg.value = sensedValue
        sensorPub.publish(smsg)

        # Particle filter: move the particles for simulating the anomaly's dynamics
        pf.move_particles()


        # Sensed values
        samples = []
        # FIXME this variable cotain all robots (even o mrobot)
        orobots = []
        for msg in robot_msgs.values():
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
                anomaly_points.append(an)
                an.robot_id = msg.robot_id

        # Particle filter: updade based on sensor value.
        pf.update_particles(samples)

        # Particle filter: Resampling.
        pf.resample()

        # Publish particles
        msg_parts = Particle()
        msg_parts.particles = pf.particles
        mrobot = PointR()
        mrobot.x, mrobot.y, mrobot.z = robotX, robotY, robotT
        msg_parts.mrobot = mrobot
        msg_parts.orobots = orobots
        msg_parts.anomaly = anomaly_points
        partPub.publish(msg_parts)


        ##### Plan in grid ####
        # Get a matrix with the number of particles for each cell.
        #grid = pf.particles_in_grid()

        # Convert sensor position to grid cell
        spi, spj = coords_to_grid(camX, camY)


        # Where to navigate
        goalX = None
        goalY = None

        #### Evaluate Sensed value
        # The flag for exploring change only if  the robot does not
        # sense an anomaly in a n seconds (n = max_tracking_time).
        if sensedValue > 0:
            explore = False
            last_time_anomaly = rospy.get_rostime().secs
        else:
            if rospy.get_rostime().secs - last_time_anomaly > max_tracking_time:
                explore = True


        ######## Exploring #################
        if explore:
            k_groups = 9
            #
            x, y = [], []
            for p in pf.particles:
                x.append(p.x)
                y.append(p.y)

            # Categorize in k groups and compute
            # new centroids
            np.random.seed(1)

            if (k_skip < 0):
                cents, idx = kmeans2(np.array(zip(x, y)), k_groups)
                k_skip = 5
            else:
                k_skip -= 1



            # Total force
            F = [0, 0]
            # Foces by the clusters
            fc = []
            for i in range(len(cents)):
                c = cents[i]

                # points in centroid
                n_pts = sum(idx == i)

                # Vector to the centroid
                d, theta = points_to_vector([robotX, robotY], c)

                # Force. Coulomb law. Charge c=n_pts
                fm = f_centroid * n_pts / (d ** 2)

                # components
                u, v = vector_components(fm, theta)

                F[0] += u
                F[1] += v

                fc.append([u, v])


            ## Foces by other robots
            fr = []
            try:
                for r in robot_msgs.values():
                    if (r.robot_id == robotName):
                        continue

                    # Vector to the other robot
                    d, theta = points_to_vector([robotX, robotY], [r.rx, r.ry])

                    # Foce, Coulombs law. Charge c=n_pts
                    k = f_robots * (len(pf.particles) / len(cents))
                    fm = k / (d ** 2)

                    # Components
                    u, v = vector_components(fm, theta)

                    # Positive or robot Force is in opposite direction.
                    F[0] -= u
                    F[1] -= v

            except Exception, e:
                rospy.logerr("Error integrating the data from other robots. " + str(e))

            cte = 10.0 / (len(pf.particles))
            #cte = 0.5 * 1 / hypot(F[0] , F[1])
            F = cte * F[0], cte * F[1]

            print "----------Total force: ", F

            goal = robotX + F[0], robotY + F[1]

            # Publish goal to navigate
            p = Point32()
            p.x, p.y = goal
            navPub.publish(p)

        else:
            ####### Tracking ##########
            controlP = (sensedLeft - 1) + sensedRight
            #print "l=", sensedLeft, " r=",sensedRight, " control=", controlP

            # counter robot force.
            crf = 0

            ### TODO Observe the other robots
            for r in robot_msgs.values():
                if (r.robot_id == robotName):
                    continue

                # Vector to the other robot
                d, theta = points_to_vector([robotX, robotY], [r.rx, r.ry])
                # Robot force.
                rf = 1 / (d ** 2)
                print [r.rx, r.ry]  #, "d=",d," theta=", theta
                # is this robot considerable?
                # if the other robot is in front of it (angle view is 120 degress)
                if abs(theta - robotT) < (pi / 3):
                    if rf > crf:
                        crf = rf

            p = Point32()
            p.x = controlP
            p.y = crf
            #print "f=", crf
            trackPub.publish(p)

        rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
