#!/usr/bin/env python
import os
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import rospy

# rostopic pub -r 20 /gazebo/set_model_state gazebo_msgs/ModelState
# '{model_name: coke_can, pose: { position: { x: 1, y: 0, z: 2 },
# orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } },
# twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'

#
SDF_FILE = '/home/dav/.gazebo/models/1fogo/model.sdf'
SPAWN_COMMAND = 'rosrun gazebo_ros spawn_model -file {0} -sdf -model {1} -x {x} -y {y}'


def spawn_fire(id, x, y):
    cmd = SPAWN_COMMAND.format(SDF_FILE, id, x=x, y=y)
    os.system(cmd)


def simulate_anomaly():
    """
    Spawn and move the fire
    """
    # {id: [ initial_pos, speed]}
    anomalies = {'fogo4': [[-0.50, 0.30], [0.0051, 0.0]],
                 'fogo5': [[-0.4, -0.30], [0.0051, 0.0]],
                'fogo6': [[-0.65, 0.2], [0.0051, 0.0]],
                'fogo7': [[0.5, 0], [0.0051, 0.0]],
                'fogo8': [[0, 0.4], [0.0051, 0.0]],
                'fogo9': [[0, -0.35], [0.0051, 0.0]],
                'fogo1': [[0.40, 0.40], [0.0051, 0.0]],
                'fogo2': [[-0.8, 0.0], [0.0051, 0.0]],
    }

    # ## SPAWN
    for anom, data in anomalies.items():
        spawn_fire(anom, data[0][0], data[0][1])

    # ### MOVE
    service_name = '/gazebo/set_model_state'
    rospy.init_node('anomaly_simulator')
    rospy.wait_for_service(service_name)

    client_ms = rospy.ServiceProxy(service_name, SetModelState)

    rospy.sleep(0.1)

    r = rospy.Rate(10)  # 10hz
    #
    # # moving_x = 0
    while not rospy.is_shutdown():

        for anom, data in anomalies.items():
            # Position + V
            data[0][0] += data[1][0]
            data[0][1] += data[1][1]

            # Model state
            ms = ModelState()
            ms.model_name = anom
            ms.pose.position.x = data[0][0]
            ms.pose.position.y = data[0][1]
            ms.reference_frame = 'world'

            # Set new model state
            set_ms = SetModelState()
            set_ms.model_state = ms
            client_ms.call(ms)
            rospy.sleep(0.05)

        r.sleep()


if __name__ == '__main__':
    try:
        simulate_anomaly()
    except rospy.ROSInterruptException:
        pass

