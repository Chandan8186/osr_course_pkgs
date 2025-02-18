import numpy as np
import pylab as pl
from scipy.spatial import KDTree
import heapq
import prm_2d
import random
import sys
sys.path.append('../osr_examples/scripts/')
import environment_2d

# adjust parameters here
obstacle_count = 5
number_of_nodes = 500
random_seed = 71

def main():
    np.random.seed(random_seed)
    env = environment_2d.Environment(10, 6, obstacle_count)
    prm = prm_2d.PRM(env, n_nodes=number_of_nodes, k_neighbors=10, max_edge_length=2.0)

    prm.sample()
    prm.construct_roadmap()

    pl.figure(figsize=(10, 6))
    prm.plot_roadmap()

    q = env.random_query()
    if q is not None:
        x_start, y_start, x_goal, y_goal = q
        path = prm.find_path((x_start, y_start), (x_goal, y_goal), prm.heuristic_a_star)
        path = prm.do_path_shortcutting(path)

        prm.trace_path(path)
        if path:
            print()
            print(f"Total path length: {round(prm.find_path_cost(path), 3)}")
            print()
        else:
            print()
            print(f"No path found.")
            print()
        env.plot_query(x_start, y_start, x_goal, y_goal)

    pl.show(block=True)

if __name__ == '__main__':
    main()
    