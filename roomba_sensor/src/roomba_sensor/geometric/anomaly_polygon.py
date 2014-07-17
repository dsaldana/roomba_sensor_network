from roomba_sensor.geometric import polygon
from roomba_comm.msg import PointR


class AnomalyPolygon(object):
    _SIMPLIFY_TH = 0.01

    def __init__(self, id_robot):
        self.id_robot = id_robot
        self.anomaly_lines = {}
        self.main_line = []

    def add_anomaly_point(self, robot_id, point):
        """
        Add a new anomaly point to the main_line or to
        the anomaly lines for other robots.
        :param robot_id: robot identification
        :param point: tuple (x,y)
        """
        if robot_id == self.id_robot:
            self.main_line.append(point)

            # ## Simplify poly line.
            self.main_line = polygon.simplify_polygon(self.main_line, self._SIMPLIFY_TH)
            return

        # Add line to other robots
        if robot_id not in self.anomaly_lines:
            self.anomaly_lines[robot_id] = []

        self.anomaly_lines[robot_id].append(point)
        # Simplify
        self.anomaly_lines[robot_id] = polygon.simplify_polygon(self.anomaly_lines[robot_id], self._SIMPLIFY_TH)

        # print self.anomaly_lines[robot_id]
        return

    def process(self):

        # Other robots
        robots_anomaly = []
        ## Check if other lines are near to fuse
        for robot_i, line in self.anomaly_lines.iteritems():

            # Check for collision
            ### Fuse
            fused_line = polygon.lines_fusion(self.main_line, line)
            # if was fused
            if self.main_line != fused_line:
                robots_anomaly.append(robot_i)
                self.main_line = fused_line
                self.anomaly_lines[robot_i] = fused_line

        print robots_anomaly

        ## Check if main_line closes
        first_point = polygon.identify_first_point_in_polygon(self.main_line, ddd=0.4)

        ## Check if the main_line closes
        closed_polygon = first_point >= 0

        if closed_polygon:
            print "Closed polygon!!!", first_point
            # # Cut the closed point or leave equal.
            self.main_line = self.main_line[first_point:]
        # else:
        #     #     print first_point, len(self.main_line)
        #     print "no closed"

    def get_anomaly_points(self):
        anomaly_points = []
        for p in self.main_line:
            an = PointR()
            an.x, an.y = p
            anomaly_points.append(an)

        return anomaly_points