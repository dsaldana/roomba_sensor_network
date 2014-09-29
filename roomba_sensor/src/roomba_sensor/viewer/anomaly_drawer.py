"""
Anomaly to be projected by a video-beam.
For physical robots.
"""
# ########### Anomaly definition##################
begin_anomalies = {
    # 'fogo1': {"loc": [-2.0, -2.0], 'target': [-2.0, -2.0], 'start_t': 0},
    # 'fogo11': {"loc": [-2.0, -2.0], 'target': [-2.0, -1.5], 'start_t': 0},
    # 'fogo12': {"loc": [-2.0, -2.0], 'target': [-2.5, -1.0], 'start_t': 0},
    # 'fogo13': {"loc": [-2.0, -2.0], 'target': [-2.5, -0.5], 'start_t': 0},
    # 'fogo14': {"loc": [-2.0, -2.0], 'target': [-1.5, -0.5], 'start_t': 0},
    # 'fogo15': {"loc": [-2.0, -2.0], 'target': [-2.0, -0.0], 'start_t': 0},
    # 'fogo16': {"loc": [-2.0, -2.0], 'target': [-2.5, -0.0], 'start_t': 0},
    # 'fogo17': {"loc": [-2.0, -2.0], 'target': [-2.0, 0.3], 'start_t': 0},
    'fogo2': {"loc": [2.0, 2.0], 'target': [2.0, 2.0], 'start_t': 0},
    'fogo21': {"loc": [2.0, 2.0], 'target': [2.0, 1.5], 'start_t': 0},
    'fogo22': {"loc": [2.0, 2.0], 'target': [2.5, 1.0], 'start_t': 0},
    'fogo23': {"loc": [2.0, 2.0], 'target': [2.5, 0.5], 'start_t': 0},
    'fogo24': {"loc": [2.0, 2.0], 'target': [2.5, 0.0], 'start_t': 0},
    # 'fogo25': {"loc": [2.0, 2.0], 'target': [2.0, 0.0], 'start_t': 0},
    # 'fogo26': {"loc": [2.0, 2.0], 'target': [2.5, 0.0], 'start_t': 0},
    # 'fogo27': {"loc": [2.0, 2.0], 'target': [2.0, -0.3], 'start_t': 0},

}
vel = 4.2
scale = 1.
dt = 0.001
# ######################################################

# For plotting
import copy
import pygame
from pygame.locals import *
import numpy as np
# Window size
import math

width, height = 750, 600

# The map is represented by a rectangle from (x1,y1) to (x2,y2)
mapX1 = -5
mapX2 = 5
mapY1 = -5
mapY2 = 5
# Map size
mapLX = mapX2 - mapX1
mapLY = mapY2 - mapY1
# Grid size
gn = 10  # Number of rows
gdx = mapLX / gn  # delta x
gm = 10  # Number of columns
gdy = mapLY / gm  # delta y


class Drawer(object):
    """
    Simple viewer for drawing.
    """

    def __init__(self):
        pygame.init()
        self.window = pygame.display.set_mode((width, height), HWSURFACE | DOUBLEBUF | RESIZABLE)
        self.update_constants()

    def set_title(self, title):
        pygame.display.set_caption(title)

    def _convert_axis_p(self, p):
        x2, y2 = self._convert_axis(p[0], p[1])
        return [x2, y2]

    def _convert_axis(self, x, y):
        x2 = int(self.mx + (x - mapX1) * self.ax / mapLX)
        y2 = int(self.my + (-y - mapY1) * self.ay / mapLY)

        return x2, y2

    def clear(self, draw_grid=True, background_color=(255, 255, 255)):
        """
        Draw the particles using  pygame.
        :return:
        """
        # Grid constats
        self.update_constants()

        # Draw the canvas
        self.window.fill(background_color)

        ilines = range(gn + 1)
        jlines = range(gm + 1)

        if not draw_grid:
            ilines = [0, len(ilines) - 1]
            jlines = [0, len(jlines) - 1]
        else:
            # Draw rows
            color = (170, 170, 170)
            for i in ilines:
                hr = self.my + i * self.dy
                pygame.draw.line(self.window, color, (self.mx, hr), (width - self.mx, hr))
            # Draw columns
            for j in jlines:
                hc = self.mx + j * self.dx
                pygame.draw.line(self.window, color, ( hc, self.my), (hc, height - self.my))

            # Draw zeros
            if draw_grid:
                pygame.draw.line(self.window, (0, 0, 250), (self.mx / 2, self.my + self.dy * gn / 2),
                                 (width - self.mx / 2, self.my + self.dy * gn / 2))
                pygame.draw.line(self.window, (0, 0, 250),
                                 (self.mx + self.dx * gm / 2, self.my / 2),
                                 (self.mx + self.dx * gm / 2, height - self.my / 2))

    def update_constants(self):
        # Window size
        try:
            width, height = self.window.get_size()
        except:
            return
        # Margin
        self.mx = width / 12.0
        self.my = height / 12.0

        self.ax = width - 2.0 * self.mx  # area for x
        self.ay = height - 2.0 * self.my  # area for y
        self.dx = self.ax / gm  # Delta x
        self.dy = self.ay / gn  # Delta y

    def draw_path(self, points, color=(255, 0, 0), stroke=2):
        """

        :param points:
        :param color:
        :return:
        """
        for i in range(1, len(points)):
            # draw line
            self.draw_line(points[i - 1], points[i], color, stroke)


    def draw_line_angle(self, p1, angle, distance, color=(255, 0, 0), stroke=2):


        p2 = p1[0] + math.cos(angle) * distance, p1[1] + math.sin(angle) * distance
        self.draw_line(p1, p2, color, stroke)


    def draw_line(self, p1, p2, color=(255, 0, 0), stroke=2):
        """
        Draw a line from p1 to p2
        :param p1:
        :param p2:
        :param color:
        :param stroke:
        """
        p1, p2 = self._convert_axis_p(p1), self._convert_axis_p(p2)
        pygame.draw.line(self.window, color, p1, p2, stroke)


    def draw_polygon(self, points, color=(255, 0, 0), stroke=2):
        """
        Draw a polygon based on points
        :param points:
        :param color:
        :param stroke: stroke of the polygin, 0=fill
        """
        pts = [self._convert_axis_p(p) for p in points]
        pygame.draw.polygon(self.window, color, pts, stroke)

    def draw_circle(self, point, radio=2, stroke=0, color=(0, 255, 0)):
        radio = int(radio * self.dx)

        p = self._convert_axis_p(point)
        pygame.draw.circle(self.window, color, p, radio, stroke)

    def draw_circles(self, points, color=(0, 0, 255), radio=2, stroke=0):
        """

        :param points: p.x and p.y
        :param color:
        :return:
        """
        for p in points:
            self.draw_circle(p, color, radio, stroke)


    def draw(self):
        pygame.display.flip()


if __name__ == '__main__':
    import time

    v = Drawer()

    # Anomaly
    pol_x = np.array(
        [0.3307448, 0.1881133, 0.01536923, -0.2612961, -0.5670846, -0.8093343, -0.9263469, -0.9586499, -0.8521932,
         -0.711869, -0.6196272, -0.5643526, -0.5131598, -0.4361191, -0.1427335, 0.1538811, 0.4093788, 0.6108048,
         0.7479919, 0.8972572, 1.060132, 1.142605, 1.13548, 1.089851, 1.197568, 1.273284, 1.315749, 1.326829, 1.241316,
         1.050877, 0.7556247, 0.4828332])
    pol_y = np.array(
        [1.019456, 0.9131819, 0.7809118, 0.6681452, 0.5878007, 0.5015554, 0.3559723, 0.1410755, -0.1227812, -0.3687398,
         -0.5366933, -0.650959, -0.7255246, -0.7652861, -0.8090969, -0.8735044, -0.9811091, -1.053721, -1.032027,
         -0.9932546, -0.9223226, -0.752345, -0.5245203, -0.2309033, -0.05224478, 0.1407709, 0.3651305, 0.6187087,
         0.8410394, 1.005029, 1.093469, 1.086207])




    # scale
    pol_x *= scale
    pol_y *= scale

    t = 0
    anomalies = copy.deepcopy(begin_anomalies)

    # Create polygons
    def restart():
        for key, vals in begin_anomalies.items():
            anomalies[key]['pol_x'] = pol_x[:] + vals['loc'][0]
            anomalies[key]['pol_y'] = pol_y[:] + vals['loc'][1]
            anomalies[key]['loc'] = vals['loc'][:]

    restart()

    _quit = False
    while not _quit:
        # Pygame chars
        for e in pygame.event.get():
            if e.type is KEYDOWN != 0:
                print "restart"
                restart()
                t = 0
            if e.type is QUIT:
                _quit = True
            if e.type is KEYDOWN and e.key == K_ESCAPE:
                _quit = True


        # Clear screen
        v.clear(background_color=(0, 0, 0), draw_grid=False)



        # Draw anomalies
        for key, vals in anomalies.items():
            if vals['start_t'] > t:
                continue

            lx, ly = vals['loc']
            tx, ty = vals['target']

            angle = math.atan2(ty - ly, tx - lx)

            dx, dy = vel * math.cos(angle) * dt, vel * math.sin(angle) * dt
            vals['loc'][0] += dx
            vals['loc'][1] += dy

            # polygon
            vals['pol_x'] += dx
            vals['pol_y'] += dy

            v.draw_polygon(zip(vals['pol_x'], vals['pol_y']), stroke=0)

        t += 1
        v.draw()
        time.sleep(dt)
