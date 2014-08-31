from shapely.geometry import Polygon, Point

from roomba_sensor.geometric import polygon


class PolygonJoiner(object):
    def __init__(self, data_polygons):
        self.open_area = Polygon()
        self.full_area = Polygon()

        for id_robot, pol_data in data_polygons.items():
            # pol_data = [polygon, closed, time]
            #fixme
            pol_data = polygon.fix_polygon(pol_data)

            try:
                #fixme
                pol_data = polygon.fix_polygon(pol_data)

                full = pol_data[1]
                pol = Polygon(pol_data[0])

                # print "full,p2=", self.full_area, ",", pol
                if full:
                    self.full_area = self.full_area.union(pol)
                else:
                    self.open_area = self.open_area.union(pol)

            except Exception:
                print "Error Joining polygons", SystemError.message

    def point_in_open_anomaly(self, point):
        p = Point(point)
        return p.within(self.open_area)
        pass

    def point_in_full_anomaly(self, point):
        p = Point(point)
        return p.within(self.full_area)