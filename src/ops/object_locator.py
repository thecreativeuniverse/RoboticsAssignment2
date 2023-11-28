import numpy as np
import matplotlib.pyplot as plt


def norm_pdf(x, mean=0, var=0.5):
    multiplier = 10 if mean == 0 else 1
    return (1 / np.sqrt(2 * np.pi * var)) * np.exp(-((x - mean) ** 2) / (2 * var)) * multiplier


def get_prob(current_x, current_y, known_dists):
    prob = 0
    for ((known_x, known_y), mean, var) in known_dists:
        distance = np.sqrt((current_x - known_x) ** 2 + (current_y - known_y) ** 2)
        prob += (norm_pdf(distance, mean, np.sqrt(var)))
    return 0 if len(known_dists) == 0 else prob / len(known_dists)


# Generate probs
def calculate_likelihoods(simple_map=None, target=None, srg=None, known_obj_locs=None):
    res = []
    known_dists = srg.get_target_distribution(target)
    distributions = []
    for obj, (x, y) in known_obj_locs:
        mean, var, _ = known_dists.get(obj)
        distributions.append(((x, y), mean, var))
    map_size = round(len(simple_map[0]) / 20)
    for x in range(-map_size, map_size):
        for y in range(-map_size, map_size):
            prob_z = get_prob(x, y, distributions)
            for m in range(10):
                i = (x + m, y + m, prob_z)
                res.append(i)
    return res


def get_weight(particle=None, target=None, srg=None, known_obj_locs=None):
    known_dists = srg.get_target_distribution(target)
    return get_prob(particle.x * 20, particle.y * 20, known_obj_locs)
