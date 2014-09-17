#!/usr/bin/env python
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import rospy

# rostopic pub -r 20 /gazebo/set_model_state gazebo_msgs/ModelState
# '{model_name: coke_can, pose: { position: { x: 1, y: 0, z: 2 },
# orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } },
# twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'

def talker():
    service_name = '/gazebo/set_model_state'
    rospy.init_node('talker')
    rospy.wait_for_service(service_name)

    client_ms = rospy.ServiceProxy(service_name, SetModelState)

    r = rospy.Rate(10)  # 10hz

    moving_x = 0
    while not rospy.is_shutdown():
        set_ms = SetModelState()
        # Model state
        ms = ModelState()
        ms.model_name = 'coke_can'
        # Pose
        ms.pose.position.x = moving_x
        ms.pose.position.z = 0
        ms.reference_frame = 'world'

        set_ms.model_state = ms

        moving_x += 0.01

        # rospy.loginfo(str)

        client_ms.call(ms)
        r.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

