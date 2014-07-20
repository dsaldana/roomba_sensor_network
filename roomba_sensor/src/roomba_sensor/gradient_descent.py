import rospy
import numpy as np
# Clustering
from scipy.cluster.vq import kmeans2

from roomba_sensor.geometric.vector import points_to_vector, vector_components




class GradientDescent(object):
    _K_GROUPS = 9

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.cents = None
        self.k_skip = -1
        self.idx = None

        # Constant in Coulombs law. Force by centroid.
        self._F_CENTROID = rospy.get_param('/f_centroid', 1.0)
        self._F_ROBOTS = rospy.get_param('/f_robots', 2.0)

    def compute_forces(self, pf, robot_x, robot_y, robots):

        x, y = [], []
        for p in pf.particles:
            x.append(p.x)
            y.append(p.y)

        # Categorize in k groups and compute
        # new centroids
        np.random.seed(1)

        if self.k_skip < 0:
            self.cents, self.idx = kmeans2(np.array(zip(x, y)), self._K_GROUPS)
            self.k_skip = 5
        else:
            self.k_skip -= 1

        total_force = [0, 0]

        # Forces by the clusters
        fc = []
        for i in range(len(self.cents)):
            c = self.cents[i]

            # points in centroid
            n_pts = sum(self.idx == i)

            # Vector to the centroid
            d, theta = points_to_vector([robot_x, robot_y], c)

            # Force. Coulomb law. Charge c=n_pts
            fm = self._F_CENTROID * n_pts / (d ** 2)

            # components
            u, v = vector_components(fm, theta)

            total_force[0] += u
            total_force[1] += v

            fc.append([u, v])

        ## Forces by other robots
        try:
            for r in robots:
                if r.robot_id == self.robot_name:
                    continue

                # Vector to the other robot
                d, theta = points_to_vector([robot_x, robot_y], [r.rx, r.ry])

                # Foce, Coulombs law. Charge c=n_pts
                k = self._F_ROBOTS * (len(pf.particles) / len(self.cents))
                fm = k / (d ** 2)

                # Components
                u, v = vector_components(fm, theta)

                # Positive or robot Force is in opposite direction.
                total_force[0] -= u
                total_force[1] -= v

        except Exception, e:
            rospy.logerr("Error integrating the data from other robots. " + str(e))

        cte = 10.0 / (len(pf.particles))
        total_force = cte * total_force[0], cte * total_force[1]
        return total_force