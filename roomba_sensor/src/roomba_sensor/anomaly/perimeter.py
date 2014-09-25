from shapely.geometry.polygon import Polygon
import numpy as np



pol_x = np.array([0.3307448, 0.1881133, 0.01536923, -0.2612961, -0.5670846, -0.8093343, -0.9263469, -0.9586499, -0.8521932, -0.711869, -0.6196272, -0.5643526, -0.5131598, -0.4361191, -0.1427335, 0.1538811, 0.4093788, 0.6108048, 0.7479919, 0.8972572, 1.060132, 1.142605, 1.13548, 1.089851, 1.197568, 1.273284, 1.315749, 1.326829, 1.241316, 1.050877, 0.7556247, 0.4828332])
pol_y = np.array([1.019456, 0.9131819, 0.7809118, 0.6681452, 0.5878007, 0.5015554, 0.3559723, 0.1410755, -0.1227812, -0.3687398, -0.5366933, -0.650959, -0.7255246, -0.7652861, -0.8090969, -0.8735044, -0.9811091, -1.053721, -1.032027, -0.9932546, -0.9223226, -0.752345, -0.5245203, -0.2309033, -0.05224478, 0.1407709, 0.3651305, 0.6187087, 0.8410394, 1.005029, 1.093469, 1.086207])


def polygon_union(anomalies):
    p = Polygon()

    for position, target in anomalies.values():
        new_x = pol_x+position[0]
        new_y = pol_y+position[1]

        new_p = Polygon(zip(new_x, new_y))

        p = p.union(new_p)

    return p


def compute_perimeter(anomalies):
    return polygon_union(anomalies).boundary.length


def compute_area(anomalies):
    return polygon_union(anomalies).area


