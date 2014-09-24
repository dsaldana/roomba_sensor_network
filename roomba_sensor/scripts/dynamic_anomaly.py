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

from roomba_sensor.comm.communicator import Communicator

LOG_FILE = '/home/dav/testings_anom/log_anomalies.pkl'
SDF_FILE1 = '/home/dav/.gazebo/models/1fogo/model.sdf'
SDF_FILE2 = '/home/dav/.gazebo/models/4fogo/model.sdf'
SPAWN_COMMAND = 'rosrun gazebo_ros spawn_model -file {0} -sdf -model {1} -x {x} -y {y}'


def spawn_fire(id, x, y, file=SDF_FILE2):
    cmd = SPAWN_COMMAND.format(file, id, x=x, y=y)
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
                 'fogo2': [[-2.30, 0.20], [0.0051, 0.0]],
    }

    # {id: initial_position, target}
    anomalies = {'fogo4': [[-0.50, 0.30], [5.00, 0.0]],
                 'fogo5': [[-0.4, -1.60], [5.00, 0.0]],
                 'fogo6': [[-1.0, -2.02], [5.00, 0.0]],
                 'fogo7': [[1.0, -0.30], [5.00, 0.0]],
                 'fogo8': [[0.0, 0.4], [5.00, 0.0]],
                 'fogo9': [[-1.30, -0.90], [5.00, 0.0]],
                 'fogo1': [[0.80, -1.60], [5.00, 0.0]],
                 'fogo2': [[-0.8, 0.0], [5.00, 0.0]],
    }

    anomalies = {'fogo4': [[-0.50, 0.30], [.00, 0.0]],
                 'fogo5': [[-0.4, -1.60], [.00, 0.0]],
                 'fogo6': [[-1.0, -2.02], [.00, 0.0]],
                 'fogo7': [[1.0, -0.30], [.00, 0.0]],
                 'fogo8': [[0.0, 0.4], [.00, 0.0]],
                 'fogo9': [[-1.30, -0.90], [.00, 0.0]],
                 'fogo1': [[0.80, -1.60], [.00, 0.0]],
                 'fogo2': [[-0.8, 0.0], [.00, 0.0]],
    }


    expand=True
    if expand:
        for k,a in anomalies.items():
            anomalies[k] = a[::-1]

    communicator = Communicator('Robot1')

    # ## SPAWN
    for anom, data in anomalies.items():
        spawn_fire(anom, data[0][0], data[0][1])
        pass

    # ### MOVE
    service_name = '/gazebo/set_model_state'
    rospy.init_node('anomaly_simulator')
    rospy.wait_for_service(service_name)

    client_ms = rospy.ServiceProxy(service_name, SetModelState)

    rospy.sleep(0.1)

    r = rospy.Rate(2)  # 10hz
    #
    # # moving_x = 0
    log = []

    while not rospy.is_shutdown():

        for anom, data in anomalies.items():
            x, y = data[0]
            target_x, target_y = data[1]
            angle = math.atan2(target_y - y, target_x - x)
            v = 0.002

            data[0][0] += v * math.cos(angle)
            data[0][1] += v * math.sin(angle)

            # Model state
            ms = ModelState()
            ms.model_name = anom
            ms.pose.position.x, ms.pose.position.y = data[0]
            ms.reference_frame = 'world'

            # Set new model state
            set_ms = SetModelState()
            set_ms.model_state = ms
            client_ms.call(ms)

            rospy.sleep(0.05)

        time = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1000000000.0
        orobots, sensed_points, anomaly_polygons = communicator.read_inbox()

        log.append((time, anomaly_polygons, copy.deepcopy(anomalies),sensed_points))
        with open(LOG_FILE, 'wb') as output:
            pickle.dump(log, output, pickle.HIGHEST_PROTOCOL)
        r.sleep()


if __name__ == '__main__':
    try:
        simulate_anomaly()
    except rospy.ROSInterruptException:
        pass


