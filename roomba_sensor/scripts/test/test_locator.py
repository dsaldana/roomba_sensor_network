#!/usr/bin/env python
import rospy

from roomba_sensor.localization import RealRoomba
from roomba_comm.msg import Particle
from roomba_comm.msg import PointR


def run():
    rospy.init_node('test_locator')

    locator = RealRoomba("Robot1")

    ## for visualization
    # Create a publisher for the particles
    partPub = rospy.Publisher("/Robot1/particles", Particle)

    while not rospy.is_shutdown():
        print locator.get_position()

        if (locator.get_position() != None):
            # Publish robot position
            msg_parts = Particle()
            mrobot = PointR()
            mrobot.x, mrobot.y, mrobot.z = locator.get_position()
            msg_parts.mrobot = mrobot

            partPub.publish(msg_parts)

        rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
