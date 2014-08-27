import rospy

from roomba_sensor.geometric import polygon

PERIMETER_PER_ROBOT = 0.1

MIN_DISTANCE_POLYGON = 0.5

_SIMPLIFY_TH = 0.01

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
        self.robot_location = None

    def add_sensed_points(self, sensed_points):
        """
        Add a new anomaly point to the main_line or to
        the anomaly lines for other robots.
        :param sensed_points: position and value for each sensed point {id_robot :[x, y, th, value]}
        """
        # register the last sensed point
        if self._id_robot in sensed_points:
            self._process_sensed_value(sensed_points[self._id_robot])

        # read each sensed point
        for robot_id, sensed_point in sensed_points.items():
            #if sensed an anomaly
            if not sensed_point[3] > 0:
                continue

            point = (sensed_point[0], sensed_point[1])
            if robot_id == self._id_robot:
                self.polyline.append(point)

                # ## Simplify polyline.
                self.polyline = polygon.simplify_polyline(self.polyline, _SIMPLIFY_TH)
                return

            # Add line to other robots
            if robot_id not in self._anomaly_lines:
                self._anomaly_lines[robot_id] = []

            self._anomaly_lines[robot_id].append(point)
            # Simplify
            if len(self._anomaly_lines) > 3:
                self._anomaly_lines[robot_id] = polygon.simplify_polyline(self._anomaly_lines[robot_id],
                                                                          _SIMPLIFY_TH)

    def _process_sensed_value(self, sensed):
        """
        :param sensed [x, y, th, value]
        """
        ## Extract robot location
        self.robot_location = (sensed[0], sensed[1])

        #### Evaluate Sensed value
        # The flag for exploring change only if  the robot does not
        # sense an anomaly in a n seconds (n = max_tracking_time).
        if sensed[3] > 0:
            # if the sensed value is in a closed polygon, , there is no sensed anomaly
            if self._in_wrong_polygon():
                new_sensed = sensed[:]
                new_sensed[3] = 0.0
                return self._process_sensed_value(new_sensed)

            self.sensed_anomaly = True
            self._last_time_anomaly = rospy.get_rostime().secs
        else:
            if rospy.get_rostime().secs - self._last_time_anomaly > _MAX_TRACKING_TIME:
                self.sensed_anomaly = False
                self.polyline = []
                self.polygon_time = None
                self.is_polygon_identified = False

    def modify_polygon(self):
        """
        Modify the polygon of the anomaly.
        1. if it has its own polygon, modify it adding the new point.
        2. else, find a polygon near by other robot, own it, and modify with (1).

        """
        ## Check if main_line closes
        first_point = polygon.identify_first_point_in_polygon(self.polyline, ddd=MIN_DISTANCE_POLYGON)
        ## Check if the main_line closes
        self.is_polygon_identified = first_point >= 0

        # Validate polygon size. less than 3 points is not a polygon.
        if self.is_polygon_identified:
            if len(self.polyline[first_point:]) < 3:
                self.is_polygon_identified = False

        ## if the own polygon is closed, modify it
        if self.is_polygon_identified:
            # modify the polygon
            self.polyline = self.polyline[first_point:]
            if self.polygon_time is None:
                self.polygon_time = rospy.get_rostime()
        else:
            ## near to other polygon?
            for id_robot, pol_data in self.data_polygons.items():
                # same robot or anomaly is full.
                if self._id_robot == id_robot or pol_data[1]:
                    continue

                last_location = self.robot_location
                distance = polygon.distance_point_to_polygon(last_location, pol_data[0])

                if distance < MIN_DISTANCE_POLYGON:
                    self.polyline = polygon.fuse_point_to_polygon(last_location, pol_data[0])
                    self.is_polygon_identified = True
                    self.polygon_time = rospy.get_rostime()
                    break

                    # todo if there is not a near polygon, try with segment
                    # if not self.closed_polygon:
                    #     # Other robots
                    #     robots_anomaly = []
                    #     ## Check if other lines are near to fuse
                    #     for robot_i, line in self._anomaly_lines.iteritems():
                    #         ### try to fuse
                    #         fused_line = polygon.lines_fusion(self.polyline, line)
                    #         # if it was fused
                    #         if self.polyline != fused_line:
                    #             robots_anomaly.append(robot_i)
                    #             self.polyline = fused_line
                    #             self._anomaly_lines[robot_i] = fused_line
                    #             break

    def _in_wrong_polygon(self):
        """
        Evaluate if the robot is in a wrong polygon.

        """
        robot_location = self.robot_location

        for id_robot, pol_data in self.data_polygons.items():
            if id_robot == self._id_robot:
                continue

            # 1 for closed, [polygon, closed, time]
            if pol_data[1]:
                # Evaluate if the robot is near to a wrong polygon
                in_polygon = polygon.point_in_polygon(robot_location, pol_data[0])

                if in_polygon:
                    return True

                near_polygon = polygon.distance_point_to_polygon(robot_location, pol_data[0]) < MIN_DISTANCE_POLYGON
                if near_polygon:
                    return True

    def evaluate_anomaly_full(self):
        """
        Evaluate if the anomaly is full of robots. Report full if necessary.

        """
        # No anomaly
        if not self.is_polygon_identified:
            return False

        # how many robots are in this anomaly?
        n_in_anomaly = 1
        # take each reported anomaly
        for id_robot, pol_data in self.data_polygons.items():
            if id_robot == self._id_robot:
                continue
            # pol_data = [polygon, closed, time]
            intersect = polygon.polygons_intersect(self.polyline, pol_data[0])

            if intersect:
                n_in_anomaly += 1

        perimeter = polygon.polygon_perimeter(self.polyline)

        # print perimeter, perimeter / PERIMETER_PER_ROBOT < n_in_anomaly
        self.anomaly_full = perimeter / PERIMETER_PER_ROBOT < n_in_anomaly

        ## should I go out of the full anomaly?

    def fix_polygon(self):
        """
        Fix if the polygon is bad formed.
        """
        if len(self.polyline) < 5:
            return

        self.polyline = polygon.fix_polygon(self.polyline)
