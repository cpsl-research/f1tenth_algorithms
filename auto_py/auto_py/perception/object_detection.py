from typing import List

import numpy as np
from sklearn.cluster import DBSCAN

from .base import Cluster


class ObjectDetectionClusterer:
    """Runs a clustering algorithm to detect obstacles

    obstacles will be dense collections of points
    """

    def __init__(self):
        self.clusterer = DBSCAN(
            eps=2,
            min_samples=5,
            metric="euclidean",
            algorithm="auto",
        )

    def __call__(self, points: np.ndarray) -> List[Cluster]:

        # run clustering algorithm
        db = self.clusterer.fit(points)

        # filter the points in the clusters
        clusters = [
            Cluster(points[db.labels_ == k, :])
            for k in set(db.labels_)
            if k != -1  # -1 is just noise
        ]
        breakpoint()
        return clusters
