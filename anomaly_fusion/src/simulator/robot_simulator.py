import math
from anomaly.AnomalyPredictor import AnomalyPredictor
import numpy as np
import matplotlib.pyplot as plt
from simulator.spiral_generator import generate_spiral
from matplotlib import animation

ROBOT_RADIO = 1.5


class RobotSpiralSimulator(object):
    """
    Robot simulator for spiral movement.
    :param duration:
    :param start_time:
    :param x_lim:
    :param y_lim:
    """

    def __init__(self, duration, start_time=40, x_lim=(-40, 40), y_lim=(-40, 40),
                 show_robot=True, show_path=True, show_anomaly=True):
        ## parameter
        self.start_time = start_time

        ######### generate spiral ###################
        self.path_tracking = generate_spiral(duration)

        ### Create predictor ###
        self.ap = AnomalyPredictor()

        ##### Init for plotting ######################
        # create a simple animation
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=x_lim, ylim=y_lim)
        # anomaly circle
        self.anomaly_circle = plt.Circle((0, 0), 20, color='#ffd5d5', )
        self.ax.add_patch(self.anomaly_circle)
        # graphic robot
        self.g_robot = plt.Circle((0, 0), ROBOT_RADIO, color='black', )
        self.ax.add_patch(self.g_robot)
        self.g_robot_theta = plt.Line2D([], [], color='y', linewidth=2)
        self.ax.add_patch(self.g_robot_theta)

        # show graphic elements
        self.show_anomaly = show_anomaly
        self.show_robot = show_robot
        self.show_path = show_path

    def _animate(self, t):
        """
        Method for animation, this is called by FuncAnimation of matplotlib.
        :param t:
        :return:
        """
        # First iteration only adds old points in order to start after start_time.
        if t == 0:
            self.ap.clear_detections()
            # add old points
            for i in range(self.start_time):
                p = self.path_tracking[i]
                anomaly_time = math.hypot(p[0], p[1])  # time is proportional to the radio.
                self.ap.add_local_sensed_point(p[3], p, anomaly_time)

        # increase the time
        t += self.start_time

        ### add new measure
        p = self.path_tracking[t]  # new measure
        anomaly_radio = math.hypot(p[0], p[1])
        anomaly_time = anomaly_radio

        self.ap.add_local_sensed_point(p[3], p, anomaly_time)  # add
        # adapt polygon to the new information
        self.ap.modify_polygon(anomaly_time, ddd=20)

        # Estimated polygon
        estimated_polygon = self.ap.estimator.estimate_polygon(anomaly_time)

        ### if the polygon is identified.
        print t, anomaly_time, self.ap.is_polygon_identified, len(self.ap.polyline), len(self.ap.polyline), len(
            self.ap.estimator.vertex_path)

        ####################
        ##### Plot #########
        ####################
        self.ax.clear()
        self.ax.cla()
        self.ax.plot((-40, 40, 40, -40), (-40, -40, 40, 40))  # bounds
        # plot full path
        xx = self.path_tracking[:t + 1][:, 0]
        yy = self.path_tracking[:t + 1][:, 1]
        tt = self.path_tracking[t][2] - math.pi / 2
        path_line = self.ax.plot(xx, yy, 'y.-')
        # polygon
        polyline = np.array(self.ap.polyline)
        px, py = polyline[:, 0], polyline[:, 1]
        polygon_line = self.ax.plot(px, py, lw=2)

        # Anomaly size
        self.anomaly_circle.radius = anomaly_radio

        # Draw estimated polygon
        estimated_pol = None
        if estimated_polygon:
            estimated_polygon = np.array(estimated_polygon)
            estimated_pol = self.ax.plot(estimated_polygon[:, 0], estimated_polygon[:, 1], 'b.-')

        ## Draw robot
        self.g_robot.center = (p[0], p[1])
        # self.g_robot_theta.set_data([0, p[0]], [0, p[1]])
        self.g_robot_theta.set_data([p[0],
                                     p[0] + ROBOT_RADIO * math.cos(p[2])],
                                    [p[1],
                                     p[1] + ROBOT_RADIO * math.sin(p[2])])
        robot_patches = [self.g_robot, self.g_robot_theta]

        drawings = []
        if self.show_anomaly:
            drawings += [self.anomaly_circle]
        if self.show_path:
                drawings += path_line

        drawings += polygon_line

        ## paths
        measurement = (p[0], p[1], anomaly_time)
        if measurement in self.ap.estimator.vertex_path:
            last_path = np.array(self.ap.estimator.vertex_path[measurement])
            track_line = self.ax.plot(last_path[:, 0], last_path[:, 1], 'ro--')

            drawings += track_line




        if estimated_pol is not None:
                drawings += estimated_pol
        if self.show_robot:
            drawings += robot_patches

        return drawings

    def _get_animation(self, interval=10):
        n_frames = len(self.path_tracking) - 1 - self.start_time
        anim = animation.FuncAnimation(self.fig, self._animate, frames=n_frames,
                                       interval=interval, blit=True)
        return anim

    def show(self, interval=100):
        """
        Show the simulation.
        :param interval:
        """
        anim = self._get_animation(interval=interval)
        plt.show()

    def save(self, file_name, interval=100):
        """
        Save simulation in a file.
        :param file_name:
        :param interval:
        """
        anim = self._get_animation(interval=interval)
        anim.save(file_name)


