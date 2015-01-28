from copy import copy
from roomba_sensor.geometric.polygon import perpendicular_line_intersection, nearest_vertex


class SimpleEstimator:
    def __init__(self):
        ## For anomaly estimation
        # path for each vertex
        self.vertex_path = {}
        self.old_polygon = []

    def update_closed_path(self, nearest_vertex_idx, polyline):

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
        if nearest_point not in self.vertex_path:
            # the path of the new point is only the nearest
            self.vertex_path[sensed_location] = [nearest_point, sensed_location]
        else:
            # takes the path of the old intersected point and increases with the new one.
            old_path = self.vertex_path[nearest_point]
            self.vertex_path[sensed_location] = old_path + [sensed_location]

            # identify the nearest in old polygon
            # update the vertex_path with the last path.
            ################################

    def get_nearest_intersection(self, sensed_location, perp_theta):
        ### save the path
        intersection_point = perpendicular_line_intersection(sensed_location, perp_theta, self.old_polygon)
        nearest_point = nearest_vertex(intersection_point, self.old_polygon)

        return nearest_point



