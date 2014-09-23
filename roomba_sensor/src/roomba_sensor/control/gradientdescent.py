import rospy

from roomba_sensor.geometric.vector import points_to_vector, vector_components
from roomba_sensor.params.map import mapX2, mapX1, mapY2, mapY1


class GradientDescent(object):
    """
    Gradient descent method for robot navigation.
    """

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.k_skip = -1
        self.idx = None

        # Constant in Coulombs law. Force by centroid.
        self._F_CENTROID = rospy.get_param('/f_centroid', 1.0)
        self._F_ROBOTS = rospy.get_param('/f_robots', 1.0)
        self._F_MAP_BORDER = rospy.get_param('/f_map_border', 1.0)


    def compute_forces(self, pf, robot_x, robot_y, robots):
        """
        Computes the total force to navigate with the
        gradient descent method. Robot tries to go away from
        other robots and attracted by particles.
        :param pf: particles
        :param robot_x: x position
        :param robot_y: y position
        :param robots: other robots with r.x and r.y
        :return:
        """
        # Vector from robot to particles. distance and angle
        f_particle = self._F_CENTROID * len(robots) / 2
        fx, fy = 0, 0
        for p in pf.particles:
            # Force from robot to particles
            d, theta = points_to_vector([robot_x, robot_y], [p.x, p.y])

            # Force. Coulomb law. Charge c=n_pts
            fm = f_particle / (d ** 2)

            # New vector of force
            x, y = vector_components(fm, theta)
            fx += x
            fy += y

        # # Vector from robot to other robots

        for r in robots:
            if r.robot_id == self.robot_name:
                continue
            # force from other robot to our robot
            d, theta = points_to_vector([r.rx, r.ry], [robot_x, robot_y])

            # Constant of proportionality
            k = self._F_ROBOTS * len(pf.particles) / d
            # Force, Coulombs law. Charge per particle
            fm = k / (d ** 2)

            x, y = vector_components(fm, theta)
            fx += x
            fy += y

        # Force by border
        f_bx = 0
        if robot_x > mapX2:
            f_bx -= self._F_MAP_BORDER
        elif robot_x < mapX1:
            f_bx += self._F_MAP_BORDER
        # border in y axis
        f_by = 0
        if robot_y > mapY2:
            f_by -= self._F_MAP_BORDER
        elif robot_y < mapY1:
            f_by += self._F_MAP_BORDER

        fx += len(robots) * f_bx
        fy += len(robots) * f_by

        # TODO forces by obstacles


        # Reducing force
        cte = 10.0 / len(pf.particles)
        total_force = cte * fx, cte * fy
        return total_force