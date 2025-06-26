import numpy as np
from sensor_msgs.msg import LaserScan


class Cluster:
    def __init__(self, points: np.ndarray):
        """Points is N x D where N is num points and D is dimension"""
        self.points = points

        # compute stats on the clusters
        self.centroid = np.mean(points, axis=0)
        self.ranges = np.linalg.norm(points, axis=1)
        self.azimuths = np.arctan2(points[:, 1], points[:, 0])

    @property
    def dim(self):
        return len(self.centroid)

    @property
    def n_points(self):
        return len(self.points)

    def center(self) -> np.ndarray:
        """Return the centroid (mean) of the cluster as a 2D point."""
        return np.mean(self.points, axis=0)

    def size(self) -> int:
        return self.points.shape[0]

    def bounding_box(self) -> np.ndarray:
        x_min, y_min = np.min(self.points, axis=0)
        x_max, y_max = np.max(self.points, axis=0)
        return np.array([[x_min, y_min], [x_max, y_max]])



def laser_scan_to_points(scan_data: LaserScan):
    """Convert laser scan to numpy array of points"""
    ranges = np.array(scan_data.ranges)
    angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(ranges))

    # Handle infinite values (replace with maximum range if needed)
    ranges[np.isinf(ranges)] = scan_data.range_max

    # Calculate x and y coordinates
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    # Stack x and y coordinates to create points array
    points = np.stack((x, y), axis=-1)
    return points
