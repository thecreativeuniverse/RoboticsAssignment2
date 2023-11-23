import numpy as np
import matplotlib.pyplot as plt


def norm_pdf(x, mean=0, var=0.5):
    return (1 / np.sqrt(2 * np.pi * var)) * np.exp(-((x-mean)**2)/(2 * var))


def get_prob(current_x, current_y, known_dists):
    prob = 0
    for ((dist_x, dist_y), dist_distance, dist_var) in known_dists:
        distance = np.sqrt((current_x - dist_x) ** 2 + (current_y - dist_y) ** 2)
        prob += (norm_pdf(distance, dist_distance, dist_var))
    return 0 if len(known_dists) == 0 else prob / len(known_dists)


# Generate probs
def calculate_likelihoods(simple_map, target, srg, known_obj_locs):
    res = []
    known_dists = srg.get_target_distribution(target)
    if type(known_obj_locs) is not list:
        known_obj_locs = [known_obj_locs]
    distributions = []
    for obj, (x, y) in known_obj_locs:
        if obj == target:
            continue
        mean, var, _ = known_dists.get(obj)
        distributions.append(((x, y), mean, var))
    map_size = round(len(simple_map[0]) / 2)
    for x in range(-map_size, map_size):
        for y in range(-map_size, map_size):
            prob_z = get_prob(x, y, distributions)
            i = (x, y, prob_z)
            res.append(i)
    print(max(res, key=lambda thing:thing[-1]))
    return res
