from typing import List

import numpy as np

from .base import Cluster


class ObjectDetectionClusterer:
    """Runs a clustering algorithm to detect obstacles

    obstacles will be dense collections of points
    """

    def __call__(self, points: np.ndarray) -> List[Cluster]:

        # TODO: run algorithm on points to get clusters

        return []
