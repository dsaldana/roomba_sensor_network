### Source: http://stackoverflow.com/questions/2573997/reduce-number-of-points-in-line
import math

from shapely.geometry import LineString

from roomba_sensor.geometric import point_to_line


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
    :return -1 if do not close, or i if close in point i.
    """
    if len(points) < 3:
        return -1

    # last point
    lp = points[-1]

    last_d = 0
    approaching = False
    first_point = len(points) - 2

    # Distance between last point and a line segment
    distance_point_line = 2 * ddd

    while not (distance_point_line < ddd and approaching):
        # next segment
        first_point -= 1

        if first_point == -1:
            break

        pt1 = points[first_point + 1]
        pt2 = points[first_point]
        # Distance to the first point
        #FIXME DISTANCE POINT TO LINE SEGMENT
        distance_point_line = point_to_line.distance_point_to_line_segment(lp, pt1, pt2)

        dist = math.hypot(lp[0] - pt2[0], lp[1] - pt2[1])
        # print last_d, dist, approaching
        approaching = dist < last_d
        last_d = dist

    print distance_point_line

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
        per += math.hypot(points[i - 1][0] - points[i][0], points[i - 1][1] - points[i][1])

    return per


def bounding_box(line1):
    """
    Bounding box for a polyline.
    :param line1: polyline defined by a set of points (tuples x,y)
    :return: two points: (min_x, min_y), (max_x, max_y)
    """
    x1 = [i[0] for i in line1]
    y1 = [i[1] for i in line1]

    min_x = min(x1)
    max_x = max(x1)
    min_y = min(y1)
    max_y = max(y1)

    return (min_x, min_y), (max_x, max_y)


def polyline_collision(line1, line2):
    ### Bounds-rectangle collision
    p1, p2 = bounding_box(line1)
    p3, p4 = bounding_box(line2)


def lines_fusion(line1, line2):
    """
    Validate each line segment for intersection. Intersection is checked as
    http://en.wikipedia.org/wiki/Line_segment_intersection

    :param line1: main line
    :param line2: line to be fused
    :return same line1 if there is no intersections with line2. else fused lines.
    """
    # TODO use bounding boxes to speed up
    # Cross validation for each line segment in polylines.
    for i in range(len(line1) - 1, 1, -1):
        p1 = line1[i]
        p2 = line1[i - 1]
        # line segment
        l1 = LineString([p1, p2])

        for j in range(len(line2) - 1, 0, -1):
            p3 = line2[j]
            p4 = line2[j - 1]

            l2 = LineString([p3, p4])
            # Check Line intersection
            # intersection = not (p1[0] - p2[0]) * (p3[1] - p4[1]) - (p1[1] - p2[1]) * (p3[0] - p4[0]) == 0
            intersection = l1.intersection(l2)

            # if intersected
            if not intersection.is_empty:
                # take the line 1 from first point until the intersection and line2
                inter_p = (intersection.coords.xy[0][0], intersection.coords.xy[1][0])
                print "break", p1, p2, p3, p4, " in ", inter_p
                new_line = line2[:j] + [inter_p] + line1[i:]
                return new_line

    # no fusion
    return line1