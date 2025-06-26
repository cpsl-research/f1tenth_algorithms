import numpy as np
from sklearn.neighbors import NearestNeighbors

class ClusterTracker:
    def __init__(self, max_distance=0.5, min_samples=5, history_size=5):
        self.max_distance = max_distance  # Maximum distance to match clusters
        self.min_samples = min_samples    # Minimum number of samples for a cluster
        self.history_size = history_size  # How many past frames to consider for matching
        self.previous_centroids = []      # List to store centroids of previous frames
        self.current_ids = []             # List to store current cluster IDs
        self.cluster_counter = 0          # To assign unique IDs to clusters
        self.cluster_history = {}         # Store history of cluster positions by ID

    def _update_history(self, current_centroids, cluster_ids):
        """
        Update the history of cluster centroids for the past frames.
        """
        for i, cluster_id in enumerate(cluster_ids):
            if cluster_id not in self.cluster_history:
                self.cluster_history[cluster_id] = []
            # Add the current centroid to the history
            self.cluster_history[cluster_id].append(current_centroids[i])

            # Limit history size (only store a certain number of frames)
            if len(self.cluster_history[cluster_id]) > self.history_size:
                self.cluster_history[cluster_id].pop(0)

    def _associate_clusters(self, current_centroids):
        """
        Associates current centroids with previous centroids based on distance.
        If a cluster from the previous frame is close enough to the current one,
        it keeps its ID, otherwise, a new ID is assigned.
        
        Args:
            current_centroids (np.ndarray): Array of current cluster centroids
        
        Returns:
            np.ndarray: Array of current cluster IDs
        """
        if len(self.previous_centroids) == 0:
            # If there are no previous clusters, assign new IDs to all current clusters
            self.previous_centroids = current_centroids
            self.current_ids = np.arange(len(current_centroids))  # Using array instead of list
            return self.current_ids
        
        # Use NearestNeighbors to match clusters based on centroid proximity
        nbrs = NearestNeighbors(n_neighbors=1, radius=self.max_distance)
        nbrs.fit(self.previous_centroids)
        distances, indices = nbrs.kneighbors(current_centroids)
        
        new_ids = []
        used_indices = set()
        
        # Try to associate clusters
        for i, dist in enumerate(distances):
            if dist[0] < self.max_distance:
                prev_index = indices[i][0]
                if prev_index not in used_indices:
                    new_ids.append(self.current_ids[prev_index])
                    used_indices.add(prev_index)
                else:
                    # If already used, create a new cluster ID
                    new_ids.append(self.cluster_counter)
                    self.cluster_counter += 1
            else:
                # If no match is found, create a new ID
                new_ids.append(self.cluster_counter)
                self.cluster_counter += 1
        
        # Update history for the matched clusters
        self._update_history(current_centroids, new_ids)
        
        # Add new clusters that could not be matched
        for i in range(len(current_centroids)):
            if i not in used_indices:
                self.previous_centroids = np.vstack((self.previous_centroids, current_centroids[i]))
                self.current_ids = np.append(self.current_ids, self.cluster_counter)  # Use np.append for arrays
                self.cluster_counter += 1

        return np.array(new_ids)

    def track(self, filtered_points):
        """
        Track the clusters for the given points, and return cluster IDs
        for the current frame.
        
        Args:
            filtered_points (np.ndarray): The points in the current frame
        
        Returns:
            np.ndarray: The cluster IDs for each point
        """
        # Compute centroids of the clusters (average of all points in the cluster)
        centroids = np.array([filtered_points[filtered_points[:, 2] == i][:, :2].mean(axis=0) 
                              for i in np.unique(filtered_points[:, 2])])
        cluster_ids = self._associate_clusters(centroids)
        
        return cluster_ids
