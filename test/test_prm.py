import numpy as np
import pylab as pl
from scipy.spatial import KDTree
import heapq
import sys
import unittest

sys.path.append('../osr_examples/scripts/')
import environment_2d
sys.path.append('../PRM/')
import prm_2d


class TestCompleteness(unittest.TestCase):
    def test_path_found_when_possible_1(self):
        # test with DIFFERENT random start/goal on SAME roadmap
        np.random.seed(42)
        env = environment_2d.Environment(10, 6, 5)
        prm = prm_2d.PRM(env, n_nodes=500, k_neighbors=10, max_edge_length=2.0)

        prm.sample()
        prm.construct_roadmap()

        seeds = [47, 55, 71]
        success_count = 0

        for seed in seeds:
            np.random.seed(seed)
            q = env.random_query()
            path = None
            if q is not None:
                x_start, y_start, x_goal, y_goal = q
                path = prm.find_path((x_start, y_start), (x_goal, y_goal), prm.heuristic_a_star)
                self.assertNotEqual(path, None)
                success_count += 1

        print(f"Path is found when possible: {success_count} start-goal pairs on same roadmap")

    def test_path_found_when_possible_2(self):
        # test with DIFFERENT random roadmaps and SAME start/goal
        seeds = [47, 55, 71]
        success_count = 0
        for seed in seeds:
            np.random.seed(seed)
            env = environment_2d.Environment(10, 6, 5)
            prm = prm_2d.PRM(env, n_nodes=500, k_neighbors=10, max_edge_length=2.0)

            prm.sample()
            prm.construct_roadmap()

            np.random.seed(42)
            q = env.random_query()

            if q is not None:
                x_start, y_start, x_goal, y_goal = q
                path = prm.find_path((x_start, y_start), (x_goal, y_goal), prm.heuristic_a_star)
                self.assertNotEqual(path, None)
                success_count += 1

        print(f"Path is found when possible: {success_count} roadmaps with same start-goal pair")

    def test_no_path_returns_None(self):
        # test with impossible path
        np.random.seed(42)
        start = (2, 3)
        goal = (8, 3)
        env = environment_2d.Environment(10, 6, 0)
        obstacle = environment_2d.TriangularObstacle(5, 0, 4.5, 5, 5.5, 6) # obstacle covering entire height of env
        env.obs.append(obstacle)
        prm = prm_2d.PRM(env, n_nodes=500, k_neighbors=10, max_edge_length=2.0)

        prm.sample()
        prm.construct_roadmap()

        path = prm.find_path(start, goal, prm.heuristic_a_star)
        self.assertEqual(path, None)
        print("Obstacle splitting env into 2 disconnected components leads to no valid path")


class TestOptimality(unittest.TestCase):
    def test_path_length_error_boundary(self):
        # test with multiple node counts and same start/goal/obstacle
        # checks whether path length difference from ideal path is at most 15%
        # note that this is a rough estimate and may vary across different roadmaps
        n_nodes_arr = [500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000]
        start = (1, 1)
        goal = (9, 4)
        ideal_path_length = np.linalg.norm(np.array([1, 1]) - np.array([7, 1])) + np.linalg.norm(np.array([9, 4]) - np.array([7, 1]))
        # fixed start, goal and obstacle used for ease of visualization
        path_lengths = []

        np.random.seed(42)
        for node_count in n_nodes_arr:
            env = environment_2d.Environment(10, 6, 0)
            obstacle = environment_2d.TriangularObstacle(2, 4, 6, 5, 7, 1)
            env.obs.append(obstacle)
            prm = prm_2d.PRM(env, n_nodes=node_count, k_neighbors=10, max_edge_length=2.0)

            prm.sample()
            prm.construct_roadmap()

            path = prm.find_path(start, goal, prm.heuristic_a_star)
            path_lengths.append(prm.find_path_cost(path))
        
        path_lengths = np.array(path_lengths)
        max_error = np.max(((path_lengths - ideal_path_length) / ideal_path_length) * 100)
        print(f"Maximum error from ideal path length: {max_error}%")
        self.assertTrue(max_error < 15) # checks whether actual path length is within 15% from ideal

class TestPathShortcutting(unittest.TestCase):
    def test_path_shortcutting_gives_shorter_path(self):
        # test with DIFFERENT random roadmaps and start/goal
        seeds = [47, 55, 71]
        for seed in seeds:
            np.random.seed(seed)
            env = environment_2d.Environment(10, 6, 5)
            prm = prm_2d.PRM(env, n_nodes=500, k_neighbors=10, max_edge_length=2.0)

            prm.sample()
            prm.construct_roadmap()

            q = env.random_query()

            if q is not None:
                x_start, y_start, x_goal, y_goal = q
                path = prm.find_path((x_start, y_start), (x_goal, y_goal), prm.heuristic_a_star)
                path_cost = prm.find_path_cost(path)
                shortcut_path = prm.do_path_shortcutting(path)
                shortcut_path_cost = prm.find_path_cost(shortcut_path)

                self.assertTrue(shortcut_path_cost <= path_cost)
                # print(f"Original path cost: {round(path_cost, 5)}, shortcut path cost: {round(shortcut_path_cost, 5)}")

if __name__ == '__main__':
    unittest.main(verbosity=2)
