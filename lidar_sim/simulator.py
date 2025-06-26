from argparse import ArgumentParser
import numpy as np
from functools import partial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
from cluster_tracker import ClusterTracker


# Environment setup (walls, background)
def generate_static_environment(env_points, lidar_range):
    theta = np.random.rand(env_points) * 2 * np.pi
    r = np.random.rand(env_points) * lidar_range
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return np.vstack((x, y)).T

# Generate pedestrian point clusters
def generate_pedestrian(x, y, num_points_per_pedestrian):
    angle = 2 * np.pi * np.random.rand(num_points_per_pedestrian)
    radius = 0.3 + 0.2 * np.random.rand(num_points_per_pedestrian)
    px = x + radius * np.cos(angle)
    py = y + radius * np.sin(angle)
    return np.vstack((px, py)).T


def dbscan_cluster(points, eps, min_samples):
    model = DBSCAN(eps=eps, min_samples=min_samples)
    labels = model.fit_predict(points)
    return labels

def filter_sparse_points(points, radius, min_neighbors):
    """
    Removes points that do not have enough neighbors within a given radius.
    
    Args:
        points (np.ndarray): shape (N, 2)
        radius (float): Distance threshold
        min_neighbors (int): Minimum required neighbors (excluding self)
    
    Returns:
        np.ndarray: Filtered points
    """
    if len(points) == 0:
        return points

    nbrs = NearestNeighbors(radius=radius).fit(points)
    neighbors = nbrs.radius_neighbors(points, return_distance=False)

    mask = np.array([len(n) - 1 >= min_neighbors for n in neighbors])  # exclude self
    return points[mask]

def main(args):
    ##############################################
    # STEP 0:
    # consider ways to make the simulator and environment
    # more realistic. Add in stochasticity with noise
    # models. Try to implement more realistic models
    # of the pedestrians. Include static obstacles.
    # Think of other creative ways to make the
    # environment more realistic.
    ##############################################

    # TODO

    # Parameters
    num_pedestrians = 5
    num_points_per_pedestrian = 30
    lidar_range = 30
    frame_count = 100
    env_points = 500  # static background points
        
    # Initialize pedestrian positions and velocities
    pedestrian_positions = np.random.uniform(-10, 10, (num_pedestrians, 2))
    pedestrian_velocities = np.random.uniform(-0.1, 0.1, (num_pedestrians, 2))

    static_env = generate_static_environment(env_points, lidar_range)

    # Animation setup
    fig, ax = plt.subplots()
    sc = ax.scatter([], [], s=1)
    ax.set_xlim(-lidar_range, lidar_range)
    ax.set_ylim(-lidar_range, lidar_range)
    ax.set_aspect('equal')
    ax.set_title('2D LiDAR Simulation with Moving Pedestrians')
    tracker = ClusterTracker(max_distance=0.7, min_samples=5)

    def update(pedestrian_positions, frame):
        all_points = static_env.copy()

        # Update pedestrian positions
        pedestrian_positions[:] += pedestrian_velocities

        # Regenerate pedestrian clusters
        for pos in pedestrian_positions:
            ped_points = generate_pedestrian(*pos, num_points_per_pedestrian)
            all_points = np.vstack((all_points, ped_points))


        # NOTE: For the following tasks, you are able to use external
        # resources such as generative AI to assist you. However, I
        # expect you to take the time to understand what the code is doing
        # such that if I asked you to explain each line, you could do so.
        # Also, for steps 1 and 2, please only use "standard" packages
        # for this including:
        # numpy, scipy, sklearn, matplotlib, etc.
        # In step 3, I'll allow you to consider more advanced methods.

        ##############################################
        # STEP 1:
        # use a clustering algorithm to perform
        # detection of the pedestrians in this 2D 
        # point cloud. Do some analysis of the benefits
        # of different methods. Consider tuning the
        # parameters and perform a research study of
        # the impact of different parameters on the
        # accuracy and latency tradeoffs. Introduce
        # noise into the system to see how your
        # algorithm responds to noisy background data.
        ##############################################

        # TODO
        filtered_points = filter_sparse_points(all_points, 0.4, 5)
        labels = dbscan_cluster(filtered_points, 0.7, 18)

        ##############################################
        # STEP 2:
        # use a multi-object tracking algorithm
        # to follow the clusters over time. This 
        # should maintain consistentn identification
        # on which object is which. By following the
        # position information over time from the 
        # detections, you should be able to estimate a
        # velocity of each object as it moves in 
        # the scene. Evaluate the false positive and
        # false negative rates of your tracker.
        # Introduce noise and evaluate the tracker.
        # Consider varying the number of pedestrians.
        # What if the pedestrians cross paths?
        ##############################################

        # Add cluster labels to points for easy reference
        clustered_points = np.hstack((filtered_points, labels.reshape(-1, 1)))

        # Track clusters with the tracker
        cluster_ids = tracker.track(clustered_points)
        print(f"Cluster IDs at frame {frame}: {np.unique(cluster_ids)}")

        # Use a consistent color map for each cluster
        colors = plt.cm.get_cmap('tab10', np.max(cluster_ids) + 2)
        color_list = [colors(cluster_id) if cluster_id != -1 else (0.5, 0.5, 0.5, 0.5) for cluster_id in cluster_ids]
        

        # TODO

        ##############################################
        # STEP 3: 
        # write a motion predictor that takes 
        # a history of the estimated state of each object
        # and predicts the future trajectory forward in
        # time. Show the predicted trajectory on the
        # resulting plot in real time. Start by using
        # a simple kinematic model that only uses the
        # current position and best estimate of velocity
        # to predict into the future. Consider if there
        # are more elaborate models that could be used.
        ##############################################

        # TODO

        sc.set_offsets(filtered_points)
        sc.set_color(color_list)
        
        return sc,

    update_partial = partial(update, pedestrian_positions)
    ani = animation.FuncAnimation(fig, update_partial, frames=frame_count, interval=100, blit=True)
    plt.show()


if __name__ == "__main__":
    parser = ArgumentParser()
    args = parser.parse_args()
    main(args)