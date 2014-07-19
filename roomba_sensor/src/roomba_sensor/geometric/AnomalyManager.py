from roomba_sensor.geometric import polygon
from roomba_comm.msg import PointR


class AnomalyManager(object):
    """
    It manages the points where an anomaly is detected to create polygons
    based on local and other robot measures.
    """
    _SIMPLIFY_TH = 0.01

    def __init__(self, id_robot):
        self._id_robot = id_robot
        self._anomaly_lines = {}
        self.polyline = []
        self.closed_polygon = False

    def add_anomaly_point(self, robot_id, point):
        """
        Add a new anomaly point to the main_line or to
        the anomaly lines for other robots.
        :param robot_id: robot identification
        :param point: tuple (x,y)
        """
        if robot_id == self._id_robot:
            self.polyline.append(point)

            # ## Simplify poly line.
            self.polyline = polygon.simplify_polygon(self.polyline, self._SIMPLIFY_TH)
            return

        # Add line to other robots
        if robot_id not in self._anomaly_lines:
            self._anomaly_lines[robot_id] = []

        self._anomaly_lines[robot_id].append(point)
        # Simplify
        self._anomaly_lines[robot_id] = polygon.simplify_polygon(self._anomaly_lines[robot_id], self._SIMPLIFY_TH)

        # print self.anomaly_lines[robot_id]
        return

    def process(self):

        # Other robots
        robots_anomaly = []
        ## Check if other lines are near to fuse
        for robot_i, line in self._anomaly_lines.iteritems():

            # Check for collision
            ### Fuse
            fused_line = polygon.lines_fusion(self.polyline, line)
            # if was fused
            if self.polyline != fused_line:
                robots_anomaly.append(robot_i)
                self.polyline = fused_line
                self._anomaly_lines[robot_i] = fused_line

        print robots_anomaly

        ## Check if main_line closes
        first_point = polygon.identify_first_point_in_polygon(self.polyline, ddd=0.4)

        ## Check if the main_line closes
        self.closed_polygon = first_point >= 0

        if self.closed_polygon:
            print "Closed polygon!!!", first_point
            # # Cut the closed point or leave equal.
            self.polyline = self.polyline[first_point:]
        # else:
        #     #     print first_point, len(self.main_line)
        #     print "no closed"

    def get_anomaly_points(self):
        """
        Convert polyline to anomaly points(class PointR).

        :return: vector of PointR
        """
        anomaly_points = []
        for p in self.polyline:
            an = PointR()
            an.x, an.y = p
            anomaly_points.append(an)

        return anomaly_points