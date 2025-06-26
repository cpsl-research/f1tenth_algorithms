import numpy as np
from auto_py.perception.object_detection import ObjectDetectionClusterer


def make_points_from_clusters(
    n_clusters=3,
    cluster_sigma=2,
    pts_per_cluster=20,
    extent=40,
    pts_noise=10,
):

    # preallocate array
    points = []

    # make points in clusters
    for i in range(n_clusters):
        centroid = extent * np.random.random_sample((1, 2))
        pts = cluster_sigma * np.random.standard_normal((pts_per_cluster, 2)) + centroid
        points.append(pts)

    # add some noise
    points.append(extent * np.random.random_sample((pts_noise, 2)))

    # convert to array
    points = np.concatenate(points, axis=0)

    return points


def test_object_clusterer():
    np.random.seed(42)

    # create a couple of point clusters
    points = make_points_from_clusters(n_clusters=3)

    # run clusterer
    detector = ObjectDetectionClusterer()
    clusters = detector(points)
    assert len(clusters) == n_clusters
