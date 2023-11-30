from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN
from sklearn.metrics import silhouette_score

from sklearn.utils._testing import ignore_warnings
from sklearn.exceptions import ConvergenceWarning

import numpy as np


@ignore_warnings(category=ConvergenceWarning)
def generate_centers(item_list, min_clusters=2, max_clusters=10):
    if len(item_list) < 2:
        return item_list
    item_list.append([300, 300])

    silhouette_list = []

    for test_k in range(min_clusters, max_clusters + 1):
        kmeans = KMeans(n_clusters=test_k, n_init=10).fit(item_list)
        labels = kmeans.labels_
        silhouette_list.append(silhouette_score(item_list, labels, metric='euclidean'))

    if len(silhouette_list) == 0:
        return np.array(item_list)[item_list != [300,300]].tolist()
    actual_k = silhouette_list.index(max(silhouette_list)) + min_clusters

    kmeans = KMeans(n_clusters=actual_k, n_init=10)
    kmeans.fit(item_list)
    centers = kmeans.cluster_centers_

    return centers