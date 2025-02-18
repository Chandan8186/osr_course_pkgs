import matplotlib.pyplot as plt
import numpy as np
import pylab as pl
from scipy.spatial import KDTree
import heapq
import random

"""
import prm_2d
import sys
sys.path.append('../osr_examples/scripts/')
import environment_2d

n_nodes_arr = [500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000]
start = (1, 1)
goal = (9, 4)
ideal_path_length = np.linalg.norm(np.array([1, 1]) - np.array([7, 1])) + np.linalg.norm(np.array([9, 4]) - np.array([7, 1]))
path_lengths = []
shortcut_lengths = []


np.random.seed(70)
for node_count in n_nodes_arr:
    env = environment_2d.Environment(10, 6, 0)
    obstacle = environment_2d.TriangularObstacle(2, 4, 6, 5, 7, 1)
    env.obs.append(obstacle)
    prm = prm_2d.PRM(env, n_nodes=node_count, k_neighbors=10, max_edge_length=2.0)

    prm.sample()
    prm.construct_roadmap()

    path = prm.find_path(start, goal, prm.heuristic_a_star)
    path_lengths.append(prm.find_path_cost(path))
    path = prm.do_path_shortcutting(path)
    shortcut_lengths.append(prm.find_path_cost(path))
"""


# Note: below path lengths are obtained by running the above code.
ideal_path_length = np.linalg.norm(np.array([1, 1]) - np.array([7, 1])) + np.linalg.norm(np.array([9, 4]) - np.array([7, 1]))
n_nodes = [500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000]
path_lengths = [10.146855786909164, 10.357978331930065, 10.24503407331286, 10.142137604284033, 10.267569004588346, 10.348422077588685, 10.210856913292874, 10.200324161651572, 10.216174535120759, 10.2324247396882, 10.076025408658996, 10.079524618054931, 10.071149006072758, 10.20593882667173, 10.024653328957289, 10.314737787819391, 10.131006117455716, 10.037716976753563]
shortcut_lengths = [9.726827924241324, 9.88002053677853, 9.771756323059975, 9.753036091633081, 9.703419492333992, 9.7391899749483, 9.86917288418256, 9.89044876672206, 9.88020227430267, 9.92472672410736, 9.683335842558071, 9.65620429296213, 9.679044229037352, 9.835618171293422, 9.675643231822969, 9.73114883264919, 9.633223974438618, 9.645691040226536]

plt.plot(n_nodes, path_lengths, marker='o', linestyle='-', color='b', label='Regular Path Length')  # Customizing line style and color
plt.plot(n_nodes, shortcut_lengths, marker='o', linestyle='-', color='g', label='Shortened Path Length')  # Customizing line style and color
plt.axhline(y=ideal_path_length, color='r', linestyle=':', label='Ideal Path Length')

plt.xlabel('Number of sample points')
plt.ylabel('Path length')
plt.title('Plot of path length against number of sample points')
plt.xticks(rotation=90)

plt.legend()
plt.ylim(8, 12)

plt.grid(True)
plt.show()
