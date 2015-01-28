from copy import copy
import numpy as np

#### Parameters #####
import random
from matplotlib.pyplot import plot
import matplotlib.pyplot as plt
from anomaly.KalmanTracker import f_transition_matrix, f_offset
from anomaly.KalmanTracker import KalmanTracker

Ez = np.array([[5.51, 0], [0, 5.51]])

# measurement model
C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])





################################
######## Simulate data #########
################################

# Initial position
x = np.array([10, 20, 0, 0])
us = [[1, 1], [0, 2], [-1, 0], [1, 1], [3, 5], [-5, 2]]

# path
ztran = lambda dur: [[0, 0] for i in range(dur - 1)]
us = np.array([[1, 1]] + ztran(50) + [[-2, 1]] + ztran(60) + [[2, -2]] + ztran(20))

# real path
path = [copy(x)]

# get location with error. It simulates the sensor
get_loc = lambda: np.dot(C, x) + np.random.multivariate_normal(np.zeros(len(Ez)), Ez)

# measurements
measurements = [get_loc()]

## Simulate flight
time = []
old_t = 0
for t, u in enumerate(us):
    # if random.random() > 0.5:
    # continue

    A = f_transition_matrix(t - old_t)
    B = f_offset(t - old_t)
    x = np.dot(A, x) + np.dot(B, u)

    y = get_loc()

    measurements.append(y[:2])
    path.append(copy(x))

    time.append(t)
    old_t = t

path = np.vstack(path)
measurements = np.vstack(measurements)

# Real point locations
loc_x, loc_y = path[:, 0], path[:, 1]
meas_x, meas_y = measurements[:, 0], measurements[:, 1]




###############################################################

#### Kalman filter ####

kt = KalmanTracker()

print len(time)
path = []
for t, m in zip(time, measurements):
    x, cov = kt.estimate_state_with_measurement(1, m)
    path.append(x)

path = np.array(path)

plot(loc_x, loc_y, 'b-')
plot(meas_x, meas_y, 'g.-')

plot(path[:,0], path[:,1], 'r.-')

plt.show()