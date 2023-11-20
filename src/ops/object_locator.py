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
        prob += (norm_pdf(distance, dist_distance, dist_var))
    return prob / len(known_dists)


# Generate probs
def calculate_likelihoods(coords, known_dists):
    res = []
    for layer in coords:
        res_temp = []
        for temp_x, temp_y in layer:
            prob_z = get_prob(temp_x, temp_y, known_dists)
            i = (temp_x, temp_y, prob_z)
            res_temp.append(i)
        res.append(res_temp)
    return res
