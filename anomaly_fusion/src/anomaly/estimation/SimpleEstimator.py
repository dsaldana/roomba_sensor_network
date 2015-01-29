from copy import copy
from roomba_sensor.geometric.polygon import perpendicular_line_intersection, nearest_vertex


class SimpleEstimator:
    def __init__(self):
        ## For anomaly estimation
        # path for each vertex

        # A path associated to each vertex
        # {point: path}, where a point is (x,y) and path is a set of points.
        self.vertex_path = {}

        # Speed for each vertex
        self.vertex_speed = {}


        # set of points of the las polygon.
        self.old_polygon = []


    def replace_old_polygon(self, nearest_vertex_idx, polyline):

        # Save the old polygon before overwrite it.
        old_old_polygon = copy(self.old_polygon)

        # Save the old polygon (it does not include the last point).
        self.old_polygon = polyline[nearest_vertex_idx:-1]

        ## Remove old points in vertex_path:
        for p in old_old_polygon:
            try:
                del self.vertex_path[p]
            except KeyError:
                print "key error, deleting vertex in vertex_path"

    def update_vertex_path(self, nearest_point, sensed_location):
        # nearest point does not have a path yet.
        if nearest_point in self.vertex_path:
            # takes the path of the old intersected point and increases with the new one.
            old_path = self.vertex_path[nearest_point]
            point = sensed_location
            self.vertex_path[point] = old_path + [sensed_location]

            # identify the nearest in old polygon
            # update the vertex_path with the last path.
            ################################
        else:
            # the path of the new point is only the nearest
            self.vertex_path[sensed_location] = [nearest_point, sensed_location]


    def get_nearest_intersection(self, sensed_location, perp_theta):
        """
        Get the nearest point in the old polygon.
        :param sensed_location:
        :param perp_theta:
        :return:
        """
        intersection_point = perpendicular_line_intersection(sensed_location, perp_theta, self.old_polygon)
        nearest_point = nearest_vertex(intersection_point, self.old_polygon)

        return nearest_point



