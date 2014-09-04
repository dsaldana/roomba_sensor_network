from math import *
import random
import copy

from roomba_comm.msg import PointR

from roomba_sensor.params.map import *
from roomba_sensor.util.angle import cut_angle
from roomba_sensor.geometric.polygon_joiner import PolygonJoiner


class ParticleFilter:
    # Number of particles
    N = rospy.get_param('/particles_number', 5000)

    # Particle movement (standard deviation)
    sd_mov = rospy.get_param('/particle_movement', 0.1)

    # radio to cover
    r = rospy.get_param('/sensor_radio', 0.5)

    simulated_robots = rospy.get_param('simulated_robots', True)

    # PF weights
    _WEIGHT_NO_SENSED = 1.0
    _WEIGHT_TRACKING_LEFT = 1.3
    _WEIGHT_TRACKING_RIGHT = 0.01
    _WEIGHT_OUT_OF_MAP = 0.01
    _WEIGHT_SENSED_ZERO = 0.1

    def __init__(self, robot_name):

        self.robot_name = robot_name

        # real robots has a different behaviour (it can be fixed).
        if not self.simulated_robots:
            self._WEIGHT_TRACKING_LEFT = 0.01
            self._WEIGHT_TRACKING_RIGHT = 1.2

        # Sparse the initial particles
        self.particles = []
        for i in range(self.N):
            p = PointR()
            # Position
            p.x = random.random() * mapLX + mapX1
            p.y = random.random() * mapLY + mapY1
            # weight
            p.z = 1.0 / self.N
            self.particles.append(p)

    def move_particles(self):
        """
        Randon movement for all the particles, based on
        normal distribution

        """
        # Move the particles
        for p in self.particles:
            # Normal function
            p.x = random.normalvariate(p.x, self.sd_mov)
            p.y = random.normalvariate(p.y, self.sd_mov)

    def resample(self):

        index = int(random.random() * self.N)

        # weights
        w = [0] * self.N
        for i in range(self.N):
            w[i] = self.particles[i].z

        # resampled particles
        rp = [0] * self.N

        # resample
        beta = 0.0
        mx = max(w)
        for i in range(self.N):
            beta += random.random() * 2.0 * mx
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.N
            rp[i] = copy.deepcopy(self.particles[index])

        for p in rp:
            p.z = 1.0 / self.N

        self.particles = rp

    def update_particles(self, sensed_values, data_polygons):
        """
        Update particle weights based on sensed values.
        :param sensed_values:
        :param data_polygons:
        """
        # joint the polygons
        anomaly_area = PolygonJoiner(self.robot_name, data_polygons)

        for [senX, senY, senT, sen_val] in sensed_values:

            # Update particles
            for p in self.particles:
                # if the particles are outside the map.
                if p.x > mapX2 or p.x < mapX1 or p.y > mapY2 or p.y < mapY1:
                    p.z *= self._WEIGHT_OUT_OF_MAP
                # Particle in full polygon
                elif anomaly_area.point_in_full_anomaly((p.x, p.y)):
                    # Position
                    p.x = random.random() * mapLX + mapX1
                    p.y = random.random() * mapLY + mapY1
                    # p.z = 0
                # avaible polygon
                elif anomaly_area.point_in_open_anomaly((p.x, p.y)):
                    p.z *= self._WEIGHT_TRACKING_LEFT
                # If the particles in the robot area.
                elif sqrt((senX - p.x) ** 2 + (senY - p.y) ** 2) < self.r:
                    if sen_val == 0:
                        p.z *= self._WEIGHT_SENSED_ZERO
                    else:
                        # If the anomaly was sensed
                        # The robot follows the anomaly in counterclock
                        # direction. Then, at left has anomaly and at
                        # right has nothing.
                        left = self.evaluate_side([senX, senY], [p.x, p.y], senT)
                        if left:
                            p.z *= self._WEIGHT_TRACKING_LEFT
                        else:
                            p.z *= self._WEIGHT_TRACKING_RIGHT

                # particle was not sensed
                else:
                    p.z *= self._WEIGHT_NO_SENSED



    def particles_in_grid(self):
        """
        Particle position in grid n rows and m columns

        :return: grid with particles
        """
        grid = [[0 for j in xrange(gn)] for i in xrange(gm)]
        for p in self.particles:
            # Particle in grid
            if mapX1 < p.x < mapX2 and mapY1 < p.y < mapY2:
                i = int((p.y - mapY1) / gdy)
                j = int((p.x - mapX1) / gdx)
                grid[i][j] += 1
        return grid

    def evaluate_side(self, r, p, t):
        """
        Evaluate if the particle is in the right or left of
        the robot.
        :param r: robot position
        :param p: particle position
        :param t: robot orientation
        :return:
        """
        # Normal vector of particle
        p2 = [p[0] - r[0], p[1] - r[1]]

        pteta = atan(p2[1] / p2[0])
        if p2[0] < 0:
            pteta += pi

        pteta = cut_angle(pteta)

        result = cut_angle(pteta - t)

        return result > 0



