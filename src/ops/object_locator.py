import numpy as np
import matplotlib.pyplot as plt


def norm_pdf(x, mean=0, var=0.5, multiplier=1):
    # -1: not seen yet
    # 0: seen, no wall
    # 100: wall
    return (1 / np.sqrt(2 * np.pi * var)) * np.exp(-((x - mean) ** 2) / (2 * var)) * multiplier


def get_prob(current_x, current_y, known_dists, simple_map_val):
    prob = 0
    for ((known_x, known_y), mean, var) in known_dists:
        distance = np.sqrt((current_x - known_x) ** 2 + (current_y - known_y) ** 2)
        multiplier = 1000 if mean == 0 else 0.1 if simple_map_val == 1 else 0.01 if simple_map_val == 100 else 1
        prob += (norm_pdf(distance, mean, np.sqrt(var), multiplier))
    return 0 if len(known_dists) == 0 else prob / len(known_dists)


# Generate probs
def calculate_likelihoods(simple_map=None, target=None, srg=None, known_obj_locs=None):
    res = []
    known_dists = srg.get_target_distribution(target)
    distributions = []
    for obj, (x, y) in known_obj_locs:
        mean, var, _ = known_dists.get(obj)
        distributions.append(((x, y), mean, var))
    map_size = 250
    for x in range(-map_size, map_size):
        print(x)
        for y in range(-map_size, map_size):
            prob_z = get_prob(x, y, distributions, 0)
            for m in range(10):
                i = (x + m, y + m, prob_z)
                res.append(i)
    return res


def get_weight(particle=None, target=None, srg=None, known_obj_locs=None, simple_map=None):
    to_add = int(round(len(simple_map[0]) / 2))
    x = int(round(particle.x * 20))
    y = int(round(particle.y * 20))
    # if x + to_add > len(simple_map[0]) or x + to_add < 0 or y + to_add > len(simple_map[0]) or y + to_add < 0:
    #     return 0
    if 0 <= x + to_add < len(simple_map[0]) and 0 <= y + to_add < len(simple_map[0]):
        simple_map_val = simple_map[y+ to_add][x + to_add]
    else:
        simple_map_val = -1
    # print(x,y,to_add)
    return get_prob(x, y, known_obj_locs, simple_map_val)
