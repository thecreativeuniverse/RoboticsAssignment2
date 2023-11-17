import numpy as np
import matplotlib.pyplot as plt


def norm_pdf(x, mean=0, var=0):
    y = (x - mean) / var
    return (np.exp(-(y ** 2) / 2) / np.sqrt(2 * np.pi)) / var



def get_prob(current_x, current_y, known_dists):
    prob = 1
    if type(known_dists) is not list:
        known_dists = [known_dists]
    for ((dist_x, dist_y), dist_distance, dist_var) in known_dists:
        distance = np.sqrt((current_x - dist_x) ** 2 + (current_y - dist_y) ** 2)
        prob *= norm_pdf(distance, dist_distance, dist_var)
    return prob


# Generate probs
def calculate_likelihoods(coords, known_dists):
    res = []
    for temp_x, temp_y in coords:
        prob_z = get_prob(temp_x, temp_y, known_dists)
        i = (temp_x, temp_y, prob_z)
        res.append(i)
    return res


# # Create and plot multivariate normal distribution
# x = np.random.uniform(-15, 15, size=10000).T
# y = np.random.uniform(-15, 15, size=10000).T
#
# # ((x,y), dist, var)
# known_obj_locs_and_distributions = [((-6, 0), 5, 1), ((3, 2), 8, 3), ((1, -4), 6, 2)]
#
# xy = zip(x, y)
# res = calculate_likelihoods(xy, known_obj_locs_and_distributions)
#
# # Plot z
# x, y, z = zip(*res)
# fig, ax = plt.subplots()
# ax.scatter(x, y, c=z, cmap='cool', marker='.')
