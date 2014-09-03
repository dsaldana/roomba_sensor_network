import math

from roomba_sensor.viewer.drawer import Drawer


class RobotDrawer(Drawer):
    """
    Draw a robot
    """

    def __init__(self):
        Drawer.__init__(self)

    def draw_robot(self, position, orientation, radio=0.2, color=(40, 123, 222)):
        # Fill
        self.draw_circle(position, radio=radio, color=color)
        # Border
        self.draw_circle(position, radio=radio, color=(0, 0, 0),stroke=2)

        self.draw_line_angle(position, orientation, radio)


if __name__ == '__main__':
    import time

    v = RobotDrawer()

    c = []

    for i in range(20, 1000):
        v.clear()
        p = (i / 100.0, i / 100.0)
        c.append(p)
        v.draw_path(c)

        if i % 10 < 5:
            v.draw_circle(p, 1, stroke=2)
        else:
            v.draw_circle(p, 1.5, stroke=3)

        v.draw_robot(p, math.pi / 4)

        v.draw()
        time.sleep(0.05)
