import numpy as np
import math

def generate_spiral(duration):
    """
    Generate a robot path for tracking a growing circle. This shape is a spiral.
    :param duration:
    :return: an array of elements [x,y, theta, sensor, time]
    """
    time = np.linspace(10, duration, duration * 10)

    # with a=1 and n=4
    n = 1
    x = np.cos(time) * time ** (1. / n)
    y = np.sin(time) * time ** (1. / n)
    theta = np.arctan2(y, x) + math.pi / 2

    theta += 0.3
    ############### as log data ############3
    ## Only the path for tracking
    path_tracking = np.vstack((x, y))  # x,y
    path_tracking = np.vstack((path_tracking, theta))  # angle
    path_tracking = np.vstack((path_tracking, np.ones(len(time))))  # sensor
    path_tracking = np.vstack((path_tracking, range(len(time))))  # time
    path_tracking = path_tracking.T

    return path_tracking