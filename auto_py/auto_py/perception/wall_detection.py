from typing import List

import numpy as np

from .base import Cluster


class WallDetectionClusterer:
    """Runs a clustering algorithm to detect wall segments

    walls will be longitudinally/laterally-connected points
    """

    def __call__(self, points: np.ndarray) -> List[Cluster]:

        # TODO: run algorithm on points to get clusters

        return []
