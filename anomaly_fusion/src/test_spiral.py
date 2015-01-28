import math
from matplotlib import animation
from anomaly.AnomalyPredictor import AnomalyPredictor
import numpy as np
import matplotlib.pyplot as plt


######### generate spiral ###################
duration = 40
# Espiral en coordenadas polares
time = np.linspace(10, duration, duration * 10)

# with a=1 and n=4
n = 1
x = np.cos(time) * time ** (1. / n)
y = np.sin(time) * time ** (1. / n)
theta = np.arctan2(y, x) + math.pi / 2

############### as log data ############3
## Only the path for tracking
path_tracking = np.vstack((x, y))  # x,y
path_tracking = np.vstack((path_tracking, theta))  # angle
path_tracking = np.vstack((path_tracking, np.ones(len(time))))  # sensor
path_tracking = np.vstack((path_tracking, range(len(time))))  # time
path_tracking = path_tracking.T


######## SIMULATE ROBOT ########3

START_TIME = 82

# create a simple animation
fig = plt.figure()
ax = plt.axes(xlim=(-40, 40), ylim=(-40, 40))

### Create predictor ###
ap = AnomalyPredictor()


def animate(t):
    # initialization
    if t == 0:
        ap._clear_detections()
        # add old points
        for i in range(START_TIME):
            p = path_tracking[i]
            ap.add_local_sensed_point(p[3], p, i)

    # increase the time
    t += START_TIME

    ### add new measure
    p = path_tracking[t]  # new measure
    ap.add_local_sensed_point(p[3], p, t)  # add
    # adapt polygon to the new information
    ap.modify_polygon(time, ddd=20)

    ### if the polygon is identified.
    print t, ap.is_polygon_identified, len(ap.polyline), len(ap.polyline), len(ap.estimator.vertex_path)

    ####################
    ##### Plot #########
    ####################
    ax.clear()
    ax.plot((-40, 40, 40, -40), (-40, -40, 40, 40))  # bounds
    # plot full path
    xx = path_tracking[:t + 1][:, 0]
    yy = path_tracking[:t + 1][:, 1]
    tt = path_tracking[t][2] - math.pi / 2
    path_line = ax.plot(xx, yy, 'y.-')
    # polygon
    polyline = np.array(ap.polyline)
    px, py = polyline[:, 0], polyline[:, 1]
    polygon_line = ax.plot(px, py, lw=2)

    ## paths
    if (p[0], p[1]) in ap.estimator.vertex_path:
        # print ap.vertex_path[(p[0], p[1])]
        last_path = np.array(ap.estimator.vertex_path[(p[0], p[1])])
        track_line = ax.plot(last_path[:, 0], last_path[:, 1], 'ro--')
        return path_line + polygon_line + track_line
    else:
        # print 'not path'
        return path_line + polygon_line


# ani = animation.FuncAnimation(fig, animate, frames=len(time), interval=40, blit=True)
ani = animation.FuncAnimation(fig, animate, frames=len(time) - 1 - START_TIME, interval=100, blit=True)

plt.show()
# plt.show()
#
#
# for i in range(20):
# # increase the time
# t = i + START_TIME
#
#
#
#     ### add new measure
#     p = path_tracking[t]  # new measure
#     ap.add_local_sensed_point(p[3], p, t)  # add
#     # adapt polygon to the new information
#     ap.modify_polygon(p, time, ddd=20)
#
#     ### if the polygon is identified.
#     print ap.is_polygon_identified,
#
#
#
#
#
#     ### Plot #########
#     ax.clear()
#     ax.plot((-40, 40, 40, -40), (-40, -40, 40, 40))  # bounds
#     # plot full path
#     xx = path_tracking[:t][:, 0]
#     yy = path_tracking[:t][:, 1]
#     tt = path_tracking[t][2] - math.pi / 2
#     ax.plot(xx, yy, '.-')
#     # polygon
#     polyline = np.array(ap.polyline)
#     px, py = polyline[:, 0], polyline[:, 1]
#     ax.plot(px, py, lw=2)
#
#
#     plt.draw()


# from anomaly.AnomalyPredictor import AnomalyPredictor
#
# ### 1. detectar el primer poligono
# def identify_polygon_in_path(path_tracking):
# ap = AnomalyPredictor()
# i = None
#     # for each point in path
#     for i, p in enumerate(path_tracking):
#         time = p[4]
#         ap.add_local_sensed_point(p[3], p, time)
#
#         ap.modify_polygon(p, time, ddd=10)
#
#         if ap.is_polygon_identified:
#             return i
#     else:
#         return None
#
#
# ##########################
# from anomaly.AnomalyPredictor import AnomalyPredictor
#
#
# # create a simple animation
# fig = plt.figure()
# ax = plt.axes(xlim=(-40, 40), ylim=(-40, 40))
# ap = AnomalyPredictor()
#
# for t in range(40, 60):
#     ax.clear()
#     ax.plot((-40, 40, 40, -40), (-40, -40, 40, 40))  # bounds
#
#
#     #### plot full path
#     xx = path_tracking[:t][:, 0]
#     yy = path_tracking[:t][:, 1]
#     tt = path_tracking[t][2] - math.pi / 2
#     ax.plot(xx, yy, '.-')
#
#
#     ## measure
#     p = path_tracking[t]
#     ap.add_local_sensed_point(p[3], p, t)
#
#     ap.modify_polygon(p, time, ddd=20)
#
#     print ap.is_polygon_identified
#
#     # plt.show()
