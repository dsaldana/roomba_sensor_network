import numpy as np

import pickle

from shapely.geometry.polygon import Polygon

########### default polygons
pol_x = np.array(
    [0.3307448, 0.1881133, 0.01536923, -0.2612961, -0.5670846, -0.8093343, -0.9263469, -0.9586499, -0.8521932,
     -0.711869, -0.6196272, -0.5643526, -0.5131598, -0.4361191, -0.1427335, 0.1538811, 0.4093788, 0.6108048, 0.7479919,
     0.8972572, 1.060132, 1.142605, 1.13548, 1.089851, 1.197568, 1.273284, 1.315749, 1.326829, 1.241316, 1.050877,
     0.7556247, 0.4828332])
pol_y = np.array(
    [1.019456, 0.9131819, 0.7809118, 0.6681452, 0.5878007, 0.5015554, 0.3559723, 0.1410755, -0.1227812, -0.3687398,
     -0.5366933, -0.650959, -0.7255246, -0.7652861, -0.8090969, -0.8735044, -0.9811091, -1.053721, -1.032027,
     -0.9932546, -0.9223226, -0.752345, -0.5245203, -0.2309033, -0.05224478, 0.1407709, 0.3651305, 0.6187087, 0.8410394,
     1.005029, 1.093469, 1.086207])


def _polygon_union(anomalies, t):
    """
    merge multple polygons.
    return an array of polygons
    """
    polygons = []

    for position, target, time in anomalies.values():
        if time >= t:
            continue

        new_x = pol_x + position[0]
        new_y = pol_y + position[1]

        new_p = Polygon(zip(new_x, new_y))

        for i, cpol in enumerate(polygons):
            if cpol.intersects(new_p):
                polygons[i] = cpol.union(new_p)
                break
        else:
            polygons.append(new_p)

    merge = True

    while merge:
        merge = False
        # print polygons
        for i, p in enumerate(polygons):
            if p is None:
                continue

            for j, c in enumerate(polygons):
                if c is None:
                    continue
                if i == j:
                    continue

                if p.intersects(c):
                    polygons[i] = p.union(c)
                    polygons[j] = None
                    merge = True
                    break
    # polygons without none
    polygons = [p for p in polygons if p is not None]

    return polygons


def extract_log(file):
    """

    :param file:
    :return:
    """
    time = []
    # t vs Polygons[]
    real_anomalies = []
    # t vs anomaly_t:Detection
    detected_anomalies = []
    # t vs anomaly: req_n
    required_ns = []
    # t vs detected polygons
    tracking_robots = []

    measurements= []

    with open(file, 'rb') as evidence_input:
        log = pickle.load(evidence_input)

    # Process log
    for t, anomaly_polygons, r_anomalies, sensed in log:
        ### o = time, anomaly_polygons, anomalies, sensed
        # time
        time.append(t)
        measurements.append(sensed)
        # anomalies
        iter_real_anomalies = _polygon_union(r_anomalies, t)
        real_anomalies.append(iter_real_anomalies)

        # anomaly_polygons={robot:[polygon, closed, time, required_n]}

        # 1 poligono por cada robot
        # 1 poligono por cada anomalia
        # cuales poligonos de robot corresponden a cada anomalia y escoge el mejor
        best_pols_t = {}

        for pol_data in anomaly_polygons.values():
            # polygon in log
            try:
                log_pol = Polygon(pol_data[0])
            except:
                print "error at time ", t
                log_pol = Polygon()

            # to which anomaly this polygon corresponds
            for anom in iter_real_anomalies:
                if log_pol.intersects(anom):
                    # less time is better
                    if not anom in best_pols_t or best_pols_t[anom][2] > pol_data[2]:
                        best_pols_t[anom] = pol_data

        # Extract pol_data to Polygon
        polys_t = {}
        req_t = {}
        for key_anom, best in best_pols_t.items():
            # best=[polygon, closed, time, required_n]
            polys_t[key_anom] = Polygon(best[0])
            req_t[key_anom] = best[3]

        detected_anomalies.append(polys_t)
        required_ns.append(req_t)

        # x,y = s[0],s[1]
        tracking_robots.append(sum([s[3] > 0 for s in sensed.values()]))

    return time, real_anomalies, detected_anomalies, required_ns, tracking_robots, measurements


def get_robot_path(measurements, id_robot='Robot1'):
    path = []
    for m in measurements:
        last_point = m[id_robot][-1]

        path.append(last_point)

    return path