import math
from anomaly.AnomalyPredictor import AnomalyPredictor
import numpy as np
import matplotlib.pyplot as plt
from simulator.spiral_generator import generate_spiral
from matplotlib import animation


class RobotSpiralSimulator(object):
    """
    Robot simulator for spiral movement.
    :param duration:
    :param start_time:
    :param x_lim:
    :param y_lim:
    """

    def __init__(self, duration, start_time=40, x_lim=(-40, 40), y_lim=(-40, 40)):
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
            estimated_pol = self.ax.plot(estimated_polygon[:, 0], estimated_polygon[:, 1], 'bo-')

        ## paths
        measurement = (p[0], p[1], anomaly_time)
        if measurement in self.ap.estimator.vertex_path:
            # print ap.vertex_path[(p[0], p[1])]
            last_path = np.array(self.ap.estimator.vertex_path[measurement])
            track_line = self.ax.plot(last_path[:, 0], last_path[:, 1], 'ro--')

            drawings = [self.anomaly_circle] + path_line + polygon_line + track_line

            if estimated_pol is None:
                return drawings
            else:
                return drawings + estimated_pol

        else:
            return path_line + polygon_line

    def _get_animation(self, interval=10):
        n_frames = len(self.path_tracking) - 1 - self.start_time
        anim = animation.FuncAnimation(self.fig, self._animate, frames= n_frames,
                                       interval=interval, blit=True)
        return anim

    def show(self, interval=10):
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


