### Source: http://stackoverflow.com/questions/2573997/reduce-number-of-points-in-line
import math


def _vec2d_dist(p1, p2):
    return (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2


def _vec2d_sub(p1, p2):
    return (p1[0] - p2[0], p1[1] - p2[1])


def _vec2d_mult(p1, p2):
    return p1[0] * p2[0] + p1[1] * p2[1]


def simplify_polygon(points, gamma):
    """Does Ramer-Douglas-Peucker simplification of a curve with `dist`
    threshold.

    `line` is a list-of-tuples, where each tuple is a 2D coordinate

    Usage is like so:

    >>> myline = [(0.0, 0.0), (1.0, 2.0), (2.0, 1.0)]
    >>> simplified = simplify_polygon(myline, gamma = 2.0)
    """

    if len(points) < 3:
        return points

    (begin, end) = (points[0], points[-1]) if points[0] != points[-1] else (points[0], points[-2])

    dist_sq = []
    for curr in points[1:-1]:
        tmp = (
            _vec2d_dist(begin, curr) - _vec2d_mult(_vec2d_sub(end, begin), _vec2d_sub(curr, begin)) ** 2 / _vec2d_dist(
                begin, end))
        dist_sq.append(tmp)

    maxdist = max(dist_sq)
    if maxdist < gamma ** 2:
        return [begin, end]

    pos = dist_sq.index(maxdist)
    return (simplify_polygon(points[:pos + 2], gamma) +
            simplify_polygon(points[pos + 1:], gamma)[1:])


def identify_first_point_in_polygon(points, ddd=0.5):
    """
    Identify the cycle beginning for the last point
    :param points: array of objects of PointX
    :param ddd: distance to the fist point
    """

    # last point
    lp = points[-1]

    last_d = 0
    approaching = False
    first_point = len(points) - 1

    while not (last_d < ddd and approaching):
        first_point -= 1

        pt = points[first_point]
        #distance to the first point
        dist = math.hypot(lp.x - pt.x, lp.y - pt.y)
        # print last_d, dist, approaching
        approaching = dist < last_d
        last_d = dist

    # print len(points) - first_point
    return first_point


def perimeter(points):
    """
    Perimeter delimited by a polygon defined by a set of points.
    :param points: points are tuples of floats.
    """
    if len(points) < 3:
        return 0

    #distance between the last and the first point.
    per = 0

    for i in range(len(points)):
        per += math.hypot(points[i - 1][0] - points[i][0], points[i - 1][1]- points[i][1])

    return per

