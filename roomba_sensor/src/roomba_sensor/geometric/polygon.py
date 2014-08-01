### Source: http://stackoverflow.com/questions/2573997/reduce-number-of-points-in-line

from shapely.geometry import LineString, Point, LinearRing, Polygon


def _vec2d_dist(p1, p2):
    return (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2


def _vec2d_sub(p1, p2):
    return p1[0] - p2[0], p1[1] - p2[1]


def _vec2d_mult(p1, p2):
    return p1[0] * p2[0] + p1[1] * p2[1]


def simplify_polyline(points, gamma):
    """Does Ramer-Douglas-Peucker simplification of a curve with `dist`
    threshold.

    `line` is a list-of-tuples, where each tuple is a 2D coordinate

    Usage is like so:

    > myline = [(0.0, 0.0), (1.0, 2.0), (2.0, 1.0)]
    > simplified = simplify_polygon(myline, gamma = 2.0)
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
    return (simplify_polyline(points[:pos + 2], gamma) +
            simplify_polyline(points[pos + 1:], gamma)[1:])


def identify_first_point_in_polygon(points, ddd=0.5):
    """
    Identify the cycle beginning for the last point
    :param points: array of objects of PointX
    :param ddd: distance to the fist point ddd>0.
    :return -1 if do not close, or i if close in point i.
    """
    if len(points) < 3:
        return -1

    # last point
    lp = Point(points[-1])

    first_point = len(points) - 2

    # Distance between last point and a line segment
    distance_point_line = 2 * ddd

    while not (distance_point_line < ddd and line_perimeter(points[first_point:]) > 2):
        # next segment
        first_point -= 1

        if first_point == -1:
            break

        pt1 = points[first_point + 1]
        pt2 = points[first_point]

        # Distance to the first point to the line segment
        line_segment = LineString([pt1, pt2])
        distance_point_line = line_segment.distance(lp)

    return first_point


def fuse_point_to_polygon(point, polygon):
    """

    :param point:
    :param polygon:
    """
    p = Point(point)
    min_distance = float("inf")
    min_index = -1
    # Identify the nearest line segment.
    for i in range(len(polygon)):
        seg = polygon[i:i + 2]
        if len(seg) < 2:
            #close the polygon
            seg = [polygon[-1]] + [polygon[0]]

        line_segment = LineString(seg)
        dist = p.distance(line_segment)
        # print seg, dist
        if dist < min_distance:
            min_distance = dist
            min_index = i

    # print min_distance, min_index
    # fused_polygon
    return polygon[min_index + 1:] + polygon[:min_index + 1] + [point]


def line_perimeter(points):
    """
    Perimeter delimited by a polygon defined by a set of points.
    :param points: points are tuples of floats.
    """
    l = LineString(points)
    return l.length


def polygon_perimeter (poly):
    """
    Perimeter delimited by a polygon defined by a set of points.
    :param poly: polygon defined by a set of points (tuples of floats).
    """
    pol = Polygon(poly)
    return pol.length


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
    for i in range(len(line1) - 1, 0, -1):
        p1 = line1[i]
        p2 = line1[i - 1]
        # line segment
        l1 = LineString([p1, p2])

        for j in range(len(line2) - 1, 0, -1):
            p3 = line2[j]
            p4 = line2[j - 1]

            l2 = LineString([p3, p4])
            # Check Line intersection
            intersection = l1.intersection(l2)

            # if intersected
            if not intersection.is_empty:
                # take the line 1 from first point until the intersection and line2
                inter_p = (intersection.coords.xy[0][0], intersection.coords.xy[1][0])
                new_line = line2[:j] + [inter_p] + line1[i:]
                return new_line

    # no fusion
    return line1


def distance_point_to_polygon(point, poly):
    """
    Compute the minimum distance from a point to a polygon
    :param point:
    :param poly:
    :return: the distance
    """
    p = Point(point)
    pol = LinearRing(poly)

    return p.distance(pol)


def point_in_polygon(point, poly):
    """
    Determines if a point p is inside a polygon
    :param point:
    :param poly: polygon list of tuples (x,y)
    """
    p = Point(point)
    pol = Polygon(poly)
    return p.within(pol)


def polygons_intersect(poly1, poly2):
    """
    Identify if two polygons intersect.
    :param poly1:
    :param poly2:
    :return: true if intersection
    """
    pol1 = Polygon(poly1)
    pol2 = Polygon(poly2)
    print poly1, poly2
    return pol1.intersection(pol2).area > 0
