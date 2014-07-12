from math import *


def points_to_vector(p1, p2):
    """
    Convert two points to a vector.
    :param p1:
    :param p2:
    :return: magnitude and angle.
    """
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]

    # Magnitude. Coulomb law.
    mag = sqrt(dx ** 2 + dy ** 2)

    # Angle
    theta = atan(dy / dx)

    if dx < 0:
        theta += pi

    return mag, theta


def vector_components(mag, theta):
    """

    :param mag:
    :param theta:
    :return: Return x,y components
    """
    # components
    x, y = mag * cos(theta), mag * sin(theta)
    return x, y