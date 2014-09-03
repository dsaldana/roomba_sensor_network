import rospy

from roomba_sensor.geometric import polygon
from roomba_sensor.geometric.polygon import polyline_length

PERIMETER_PER_ROBOT = 1.1

MIN_DISTANCE_POLYGON = 0.2

# Distance to simplify polygons.
_SIMPLIFY_TH = 0.1

# Time for tracking without sensing anomaly.
_MAX_TRACKING_TIME = 5


class AnomalyManager(object):
    """
    It manages the points where an anomaly is detected to create polygons
    based on local and other robot measures.
    """

    def __init__(self, id_robot):
        self._id_robot = id_robot
        self._anomaly_lines = {}

        # When the polygon was detected
        self.polygon_time = None
        self.polyline = []
        # If the anomaly is full of robots
        self.anomaly_full = False
        self.is_polygon_identified = False
        self.sensed_anomaly = False

        # last time that an anomaly was detected
        self._last_time_anomaly = 0
        # polygons: detected anomalies {id_robot:[polygon, closed, time]}
        self.data_polygons = {}


    def add_local_sensed_point(self, sensed_value, sensed_position):
        """
        register the last sensed point for our robot.
        :param sensed_value:
        :param sensed_position:
        """
        # Wrong polygon? and update the timeout.
        self._process_sensed_value(sensed_value, sensed_position)

        self.polyline.append((sensed_position[0], sensed_position[1]))
        # ## Simplify polyline.
        # self.polyline = polygon.simplify_polyline(self.polyline, _SIMPLIFY_TH)

    def add_sensed_points(self, sensed_points):
        """
        Feed polyline for all the other robots.
        :param sensed_points: position and value for each sensed point {id_robot :[x, y, th, value]}
        """
        # read each sensed point
        for robot_id, sensed_point in sensed_points.items():
            if robot_id == self._id_robot:
                continue

            point = (sensed_point[0], sensed_point[1])

            # if not sensed an anomaly
            if not sensed_point[3] > 0:
                continue

            # if polyline for robot i was not created.
            if robot_id not in self._anomaly_lines:
                self._anomaly_lines[robot_id] = []

            # Add line to other robots
            self._anomaly_lines[robot_id].append(point)

            # Simplify
            # if len(self._anomaly_lines) > 3:
            # self._anomaly_lines[robot_id] = polygon.simplify_polyline(self._anomaly_lines[robot_id],_SIMPLIFY_TH)

    def _process_sensed_value(self, sensed_val, sensed_position):
        """
        Evaluates if the detected anomaly is in a wrong polygon.
        Also maintains the timeout for tracking the anomaly.

        :param sensed [x, y, th, value]
        """

        # ### Evaluate Sensed value
        # The flag for exploring change only if  the robot does not
        # sense an anomaly in a n seconds (n = max_tracking_time).
        if sensed_val > 0:
            # if the sensed value is in a closed polygon, , there is no sensed anomaly
            if self._in_wrong_polygon(sensed_position):
                # Treat this case as a non detection.
                sensed_val = 0.0
                return self._process_sensed_value(sensed_val)
            else:
                self.sensed_anomaly = True
                self._last_time_anomaly = rospy.get_rostime().secs
        else:
            # TIMEOUT: tracking is lost, restart variables.
            if rospy.get_rostime().secs - self._last_time_anomaly > _MAX_TRACKING_TIME:
                self.sensed_anomaly = False
                self.polyline = []
                self.polygon_time = None
                self.is_polygon_identified = False
                print "Timeout, polygon lost."

    def modify_polygon(self, sensed_location):
        """
        Modify the polygon of the anomaly.
        1. if it has its own polygon, modify it adding the new point.
        2. else, find a polygon near by other robot, own it, and modify with (1).
        :return True if the data gives a polygon.
        """
        # # Check if main_line closes
        first_point = polygon.identify_first_point_in_polygon(self.polyline, ddd=MIN_DISTANCE_POLYGON)
        polyline_closes = first_point >= 0

        # Validate polygon size. less than 3 points is not a polygon.
        if not self.is_polygon_identified:
            self.is_polygon_identified = polyline_closes
            # if len(self.polyline[first_point:]) < 3:
            # self.is_polygon_identified = False
            # else:
            ## Check if the main_line closes to create a polygon


        # # if the own polygon is closed, modify it
        if self.is_polygon_identified:
            # modify the polygon
            # FIXME only area at left
            if polyline_closes:
                self.polyline = self.polyline[first_point:]

            if self.polygon_time is None:
                self.polygon_time = rospy.get_rostime()
        else:
            # # near to other polygon?
            for id_robot, pol_data in self.data_polygons.items():
                # same robot or anomaly is full.
                if self._id_robot == id_robot or pol_data[1]:
                    continue

                distance = polygon.distance_point_to_polygon(sensed_location, pol_data[0])

                if distance < MIN_DISTANCE_POLYGON:
                    self.polyline = polygon.fuse_point_to_polygon(sensed_location, pol_data[0])
                    self.is_polygon_identified = True
                    self.polygon_time = rospy.get_rostime()
                    break

            print "not near polygon"
            # if there is not a near polygon, try with near segments
            if not self.is_polygon_identified:
                ## Check if other lines are near to fuse
                for robot_i, line in self._anomaly_lines.iteritems():

                    ### try to fuse the line segment
                    fused_line = polygon.lines_fusion(self.polyline, line)

                    print "fused line: ", polyline_length(fused_line) > polyline_length(self.polyline)
                    # if it was fused
                    if polyline_length(fused_line) > polyline_length(self.polyline):
                        self.polyline = fused_line


    def _in_wrong_polygon(self, sensed_location):
        """
        Evaluate if the robot is in a wrong polygon.

        """
        for id_robot, pol_data in self.data_polygons.items():
            if id_robot == self._id_robot:
                continue

            # 1 for closed, [polygon, closed, time]
            if pol_data[1]:
                # Evaluate if the robot is near to a wrong polygon
                in_polygon = polygon.point_in_polygon(sensed_location, pol_data[0])

                if in_polygon:
                    return True

                near_polygon = polygon.distance_point_to_polygon(sensed_location, pol_data[0]) < MIN_DISTANCE_POLYGON
                if near_polygon:
                    return True

    def evaluate_anomaly_full(self):
        """
        Evaluate if the anomaly is full of robots. Report full if necessary.
        :return True if there is no space for mor robots
        """
        # No anomaly
        if not self.is_polygon_identified:
            self.anomaly_full = False
            return

        # how many robots are in this anomaly?
        n_in_anomaly = 1
        # take each reported anomaly
        for id_robot, pol_data in self.data_polygons.items():
            if id_robot == self._id_robot:
                continue
            # pol_data = [polygon, closed, time]
            try:
                intersect = polygon.polygons_intersect(self.polyline, pol_data[0])

                if intersect:
                    n_in_anomaly += 1
            except:
                print "Error in compute intersection, to evaluete if anomaly is full"

        perimeter = polygon.polygon_perimeter(self.polyline)

        # print perimeter, perimeter / PERIMETER_PER_ROBOT < n_in_anomaly
        self.anomaly_full = perimeter / PERIMETER_PER_ROBOT < n_in_anomaly

        # # should I go out of the full anomaly?

    def fix_polygon(self):
        """
        Fix if the polygon is bad formed.
        """
        if len(self.polyline) < 5:
            return

        self.polyline = polygon.fix_polygon(self.polyline)

    def get_simplyfied_polygon(self):
        return polygon.simplify_polyline(self.polyline, _SIMPLIFY_TH)


