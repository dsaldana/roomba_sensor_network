#!/usr/bin/env python
import pickle

import rospy
import time

from roomba_sensor.comm.communicator import Communicator

#LOG_FILE = '/home/dav/testings_anom/log_anomalies.pkl'
LOG_FILE = '/home/dav/testings_anom/log_anomalies{}.pkl'.format(time.clock())


def log_anomaly():
    """
    Spawn and move the fire
    """
    communicator = Communicator('Robot1')
    rospy.init_node('log_anomaly')

    log = []

    r = rospy.Rate(2)  # 10hz
    while not rospy.is_shutdown():
        t = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1000000000.0
        print t
        orobots, sensed_points, anomaly_polygons = communicator.read_inbox()

        log.append((t, anomaly_polygons, {}, sensed_points))
        with open(LOG_FILE, 'wb') as output:
            pickle.dump(log, output, pickle.HIGHEST_PROTOCOL)
        r.sleep()


if __name__ == '__main__':
    try:
        log_anomaly()
    except rospy.ROSInterruptException:
        pass


