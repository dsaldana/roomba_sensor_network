from anomaly.log_extractor import extract_log, get_robot_path


time, real_anomalies, detected_anomalies, required_ns, tracking_robots, measurements = extract_log('/home/dav/Dropbox/notebook/kalman_filter/log_anomalies.pkl')

print measurements
