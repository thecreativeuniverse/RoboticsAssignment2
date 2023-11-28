from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score

from sklearn.utils._testing import ignore_warnings
from sklearn.exceptions import ConvergenceWarning


@ignore_warnings(category=ConvergenceWarning)
def generate_centers(item_list):
    if len(item_list)<2:
        return item_list
    item_list.append([300, 300])


    silhouette_list = []

    for test_k in range(2, len(item_list)):
        kmeans = KMeans(n_clusters=test_k, n_init=10).fit(item_list)
        labels = kmeans.labels_
        silhouette_list.append(silhouette_score(item_list, labels, metric='euclidean'))
    actual_k = silhouette_list.index(max(silhouette_list)) + 1


    kmeans = KMeans(n_clusters=actual_k, n_init=10)
    kmeans.fit(item_list)
    centers = kmeans.cluster_centers_
    for i in range(len(centers)):
        if centers[i][0] ==300:
            del centers[i]

    return centers