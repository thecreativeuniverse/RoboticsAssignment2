from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN
from sklearn.metrics import silhouette_score

from sklearn.utils._testing import ignore_warnings
from sklearn.exceptions import ConvergenceWarning

import numpy as np


@ignore_warnings(category=ConvergenceWarning)
def generate_centers(item_list, estimated_clusters):
    # dbscan = DBSCAN(eps=0.5, min_samples=2)
    # dbscan.fit(item_list)
    # labels = dbscan.labels_
    #
    # if type(item_list) is list:
    #     item_list = np.array(item_list)
    #
    # unique_labels = set(labels)
    #
    # centers = []
    #
    # for k in unique_labels:
    #     if k == -1:
    #         print("Found outliers in DBSCAN")
    #     found_obj = item_list[labels == k]
    #     if len(found_obj) == 0:
    #         print("Len found obj == 0!")
    #         continue
    #     x = [i[0] for i in found_obj]
    #     y = [i[1] for i in found_obj]
    #     ave = [sum(x) / len(x), sum(y) / len(y)]
    #     centers.append(ave)

    if len(item_list) < 2:
        return item_list
    item_list.append([300, 300])

    silhouette_list = []

    print("testing k", len(item_list), "estimated", estimated_clusters)
    for test_k in range(max(estimated_clusters - 2, 2), len(item_list)):
        kmeans = KMeans(n_clusters=test_k, n_init=10).fit(item_list)
        labels = kmeans.labels_
        silhouette_list.append(silhouette_score(item_list, labels, metric='euclidean'))
    actual_k = silhouette_list.index(max(silhouette_list)) + 2

    print("actual k", actual_k)
    kmeans = KMeans(n_clusters=actual_k, n_init=10)
    kmeans.fit(item_list)
    print("fitted")
    centers = kmeans.cluster_centers_

    return centers