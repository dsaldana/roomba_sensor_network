from math import sqrt


def euclidean_distance(p1, p2):
    """
    Compute euclidean distance for two points
    :param p1:
    :param p2:
    :return:
    """
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]

    # Magnitude. Coulomb law.
    return sqrt(dx ** 2 + dy ** 2)