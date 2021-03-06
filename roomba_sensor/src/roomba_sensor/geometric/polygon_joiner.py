from shapely.geometry import Polygon, Point
from shapely.geometry.multipoint import MultiPoint

from roomba_sensor.geometric import polygon


class PolygonJoiner(object):
    def __init__(self, robot_name, data_polygons):
        self.robot_name = robot_name
        self.open_area = Polygon()
        self.full_area = Polygon()


        for id_robot, pol_data in data_polygons.items():
            # pol_data = [polygon, closed, time]

            try:
                available = not pol_data[1]
                pol = Polygon(pol_data[0])

                # Invalid polygons cannot be join.
                if not pol.is_valid:
                    pol = MultiPoint(pol_data[0]).convex_hull

                if available or robot_name == id_robot:

                    self.open_area = self.open_area.union(pol)
                else:
                    self.full_area = self.full_area.union(pol)

            except Exception:
                print "Error Joining polygons", SystemError.message

    def point_in_open_anomaly(self, point):
        p = Point(point)
        return p.within(self.open_area)
        pass

    def point_in_full_anomaly(self, point):
        p = Point(point)
        return p.within(self.full_area)