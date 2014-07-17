import math
## Source http://www.maprantala.com/2010/05/16/measuring-distance-from-a-point-to-a-line-segment/

def _line_magnitude(x1, y1, x2, y2):
    lm = math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))
    return lm


def distance_point_to_line_segment(p, pl1, pl2):
    """
    Compute the minimum Distance between a point and a line segment.
    (i.e. consecutive vertices in a polyline).
    #http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/source.vba

    :param p: point tuple (x,y)
    :param pl1: point 1 of the line (x1, y1)
    :param pl2: point 2 of the line (x2, y2)
    :return: distance between the point and the line segment.
    """
    x1, y1 = pl1
    x2, y2 = pl2
    px, py = p

    line_mag = _line_magnitude(x1, y1, x2, y2)

    if line_mag < 0.00000001:
        return -1

    u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
    u = u1 / (line_mag * line_mag)

    if (u < 0.00001) or (u > 1):
        #// closest point does not fall within the line segment, take the shorter distance
        #// to an endpoint
        ix = _line_magnitude(px, py, x1, y1)
        iy = _line_magnitude(px, py, x2, y2)
        if ix > iy:
            d = iy
        else:
            d = ix
    else:
        # Intersecting point is on the line, use the formula
        ix = x1 + u * (x2 - x1)
        iy = y1 + u * (y2 - y1)
        d = _line_magnitude(px, py, ix, iy)

    return d


print distance_point_to_line_segment((1,1),(1,1),(2,2))