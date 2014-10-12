#!/usr/bin/env python
import copy
import os
import pickle
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import rospy
import math

# rostopic pub -r 20 /gazebo/set_model_state gazebo_msgs/ModelState
# '{model_name: coke_can, pose: { position: { x: 1, y: 0, z: 2 },
# orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } },
# twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'
from sets import Set

from roomba_sensor.comm.communicator import Communicator

LOG_FILE = '/home/dav/testings_anom/log_anomalies.pkl'

def log_anomaly():
    """
    Spawn and move the fire
    """

    existent_anomalies = Set()

    communicator = Communicator('Robot1')

    # ## SPAWN
    # for anom, data in anomalies.items():
    #     spawn_fire(anom, data[0][0], data[0][1])
    #     pass

    # ### MOVE

    rospy.init_node('log_anomaly')

    rospy.sleep(0.1)

    r = rospy.Rate(2)  # 10hz
    #
    # # moving_x = 0
    log = []

    while not rospy.is_shutdown():
        time = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1000000000.0
        print time
        orobots, sensed_points, anomaly_polygons = communicator.read_inbox()

        log.append((time, anomaly_polygons, {}, sensed_points))
        with open(LOG_FILE, 'wb') as output:
            pickle.dump(log, output, pickle.HIGHEST_PROTOCOL)
        r.sleep()


if __name__ == '__main__':
    try:
        log_anomaly()
    except rospy.ROSInterruptException:
        pass


