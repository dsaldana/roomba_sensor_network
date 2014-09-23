# Robot id
string robot_id

# Point where the robot is
float32 x
float32 y
float32 theta

# Point where the robot is
float32 rx
float32 ry
float32 rtheta

# Sensed value
float32 value
time time2

# Polygon
Point[] anomaly
# Anomaly is full of robots or not
bool closed_anomaly
# When robot detected the anomaly
time time_of_detection
# Number of required robots
int32  required_robots
