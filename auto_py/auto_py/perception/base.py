import numpy as np


class Cluster:
    def __init__(self, points: np.ndarray):
        """Points is N x D where N is num points and D is dimension"""
        self.centroid = np.mean(points, axis=0)
        self.points = points

    @property
    def dim(self):
        return len(self.centroid)

    @property
    def n_points(self):
        return len(self.points)
