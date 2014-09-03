# For plotting
import pygame
from pygame.locals import *

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

    def clear(self, draw_grid=True):
        """
        Draw the particles using  pygame.
        :return:
        """
        # Grid constats
        self.update_constants()

        # Draw the canvas
        self.window.fill((255, 255, 255))

        ilines = range(gn + 1)
        jlines = range(gm + 1)

        if not draw_grid:
            ilines = [0, len(ilines) - 1]
            jlines = [0, len(jlines) - 1]

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

    for i in range(10):
        v.clear()

        a = [(1, 1), (2, 2), (3, 1)]
        v.draw_circles(a, radio=8, stroke=2)
        v.draw_path(a)
        v.draw_circle((1, 2), radio=10, stroke=0)

        v.draw()
        time.sleep(1)