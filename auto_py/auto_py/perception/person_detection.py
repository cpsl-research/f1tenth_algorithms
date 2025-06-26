import numpy as np
from typing import List
from sklearn.cluster import DBSCAN
from .base import Cluster


class ClusterTrack(Cluster):
    # TODO: add in tracking-relevant things like
    # number of detections assigned
    # age (number of frames alive)
    # coast time (number of frames without an assignment)
    # 
    # these will be used to eliminate "dead" tracks and
    # to "confirm" tentative tracks after some established time


class ClusterTracker:
    """
    Tracks clusters across frames and assigns persistent IDs based on spatial proximity.
    """
    def __init__(self, distance_threshold: float = 1.0):
        """
        Initializes the ClusterTracker with a distance threshold for matching clusters.
        
        :param distance_threshold: Maximum distance to match clusters across frames.
        """
        self.distance_threshold = distance_threshold
        self.cluster_id_counter = 0
        self.previous_clusters: List[Cluster] = []

    def track(self, current_clusters: List[Cluster]) -> List[ClusterTrack]:
        """best_match
        Matches the current clusters with previous clusters, assigning persistent IDs.
        
        :param current_clusters: List of clusters detected in the current frame.
        :return: A list of clusters with assigned persistent IDs.
        """
        matched_clusters = []
        new_clusters = []

        # Track previous clusters by their spatial proximity
        for current_cluster in current_clusters:
            best_match = None
            min_distance = float('inf')
            for prev_cluster in self.previous_clusters:
                # Calculate the distance between the cluster centers
                distance = np.linalg.norm(current_cluster.center() - prev_cluster.center())
                if distance < self.distance_threshold and distance < min_distance:
                    best_match = prev_cluster
                    min_distance = distance

            if best_match is not None:
                # The cluster matches a previous one, keep its ID
                current_cluster.id = best_match.id
                matched_clusters.append(current_cluster)
            else:
                # New cluster, assign a new ID
                current_cluster.id = self.cluster_id_counter
                self.cluster_id_counter += 1
                new_clusters.append(current_cluster)

        # Update the previous clusters list for the next frame
        self.previous_clusters = current_clusters

        return matched_clusters + new_clusters


class DetectionClusterer:
    """
    Clustering algorithm to detect clusters without filtering out large ones.
    """
    def __init__(self):
        self.clusterer = DBSCAN(
            eps=0.5,            # tighter distance threshold
            min_samples=4,      # small group of nearby points
            metric="euclidean",
            algorithm="auto",
        )

    def __call__(self, points: np.ndarray) -> List[Cluster]:
        if points.shape[0] == 0:
            return []

        # Run DBSCAN clustering
        db = self.clusterer.fit(points)

        clusters = []
        for k in set(db.labels_):
            if k == -1:
                continue  # Ignore noise

            cluster_points = points[db.labels_ == k, :]
            clusters.append(Cluster(cluster_points))

        return clusters


class PersonDetectionClusterer(DetectionClusterer):
    """
    A thin alias class to distinguish semantically between general detection and person detection,
    built on top of DetectionClusterer.
    """
    pass
