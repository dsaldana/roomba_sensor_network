import rospy

from roomba_sensor.geometric import polygon

MIN_DISTANCE_POLYGON = 0.5


class AnomalyManager(object):
    """
    It manages the points where an anomaly is detected to create polygons
    based on local and other robot measures.
    """
    _SIMPLIFY_TH = 0.01
    # Time for tracking without sensing anomaly.
    _MAX_TRACKING_TIME = 5

    def __init__(self, id_robot):
        self._id_robot = id_robot
        self._anomaly_lines = {}
        self.polyline = []
        self.is_polygon_identified = False
        self.sensed_anomaly = False

        # last time that an anomaly was detected
        self._last_time_anomaly = 0
        # polygons: detected anomalies {id_robot:[polygon, closed, time]}
        self.data_polygons = {}

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

                # ## Simplify poly line.
                self.polyline = polygon.simplify_polygon(self.polyline, self._SIMPLIFY_TH)
                return

            # Add line to other robots
            if robot_id not in self._anomaly_lines:
                self._anomaly_lines[robot_id] = []

            self._anomaly_lines[robot_id].append(point)
            # Simplify
            self._anomaly_lines[robot_id] = polygon.simplify_polygon(self._anomaly_lines[robot_id], self._SIMPLIFY_TH)

    # def process(self):
    #
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
    #
    #     # print robots_anomaly
    #
    #     ## Check if main_line closes
    #     first_point = polygon.identify_first_point_in_polygon(self.polyline, ddd=0.4)
    #
    #     ## Check if the main_line closes
    #     self.closed_polygon = first_point >= 0
    #
    #     if self.closed_polygon:
    #         print "Closed polygon!!!", first_point
    #         # # Cut the closed point or leave equal.
    #         self.polyline = self.polyline[first_point:]
    #         # else:
    #         #     #     print first_point, len(self.main_line)
    #         #     print "no closed"

    def _process_sensed_value(self, sensed):
        """
        :param sensed [x, y, th, value]
        """
        #### Evaluate Sensed value
        # The flag for exploring change only if  the robot does not
        # sense an anomaly in a n seconds (n = max_tracking_time).
        if sensed[3] > 0:
            self.sensed_anomaly = True
            self._last_time_anomaly = rospy.get_rostime().secs
        else:
            if rospy.get_rostime().secs - self._last_time_anomaly > self._MAX_TRACKING_TIME:
                self.sensed_anomaly = False

    # def is_in_anomaly(self):
    #     ### if a robot is near to a polygon
    #     if self.last_sensed > 0:
    #         return True
    #
    #
    #     # Distance to the polygons

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
        else:
            ## near to other polygon?
            for id_robot, pol_data in self.data_polygons.items():
                # if self._id_robot == id_robot:
                #     continue

                last_location = self.polyline[-1]
                distance = polygon.distance_point_to_polygon(last_location, pol_data[0])

                if distance < MIN_DISTANCE_POLYGON:
                    self.polyline = polygon.fuse_point_to_polygon(last_location, pol_data[0])
                    self.is_polygon_identified = True
                    break

                    # if there is not a near polygon, try with segment
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

