import numpy as np
import pylab as pl
from scipy.spatial import KDTree
import heapq
import random
import sys
sys.path.append('../osr_examples/scripts/')
import environment_2d


class PRM:
    """
    A class used to represent the Probabilistic Roadmap attributes and functions

    Definitions used
    ----------------
    - E: Entire configuration space
    - E_free: Entire configuration space excluding all obstacles

    Attributes
    ----------
    env : environment_2d.Environment
        the PRM roadmap environment object
    n_nodes : int
        the number of nodes (excluding start/goal) to be randomly sampled to form the roadmap
    k_neighbors : int
        the number of nearest neighbours of a node to be selected to form edges with
    max_edge_length : float
        the maximum allowed edge length
    nodes : List[Tuple[float, float]]
        the list of coordinates of the nodes in E_free
    graph : Dict[int, List[int]]
        the mapping from a node to a list of its neighbors (nodes being represented as indices)
    kd_tree : KDTree
        the kd tree representing the graph of all nodes
    """
    
    def __init__(self, env, n_nodes=500, k_neighbors=10, max_edge_length=2.0):
        self.env = env
        self.n_nodes = n_nodes
        self.k_neighbors = k_neighbors
        self.max_edge_length = max_edge_length

        self.nodes = []
        self.graph = {}
        self.kd_tree = None

    def sample(self):
        """
        Randomly samples nodes in E_free.
        """
        self.nodes = []
        node_count = 0
        while node_count < self.n_nodes:
            x = np.random.uniform(0, self.env.size_x)
            y = np.random.uniform(0, self.env.size_y)
            if not self.env.check_collision(x, y):
                self.nodes.append((x, y))
                node_count += 1 # randomly sample a point and ensure it is in E_free
        self.kd_tree = KDTree(self.nodes) # creates a kd tree with the created nodes

    def check_edge(self, start, end):
        """
        Checks whether given edge falls within E_free.
        Small probability of false positive, can be reduced with smaller intervals than 0.01.
        
        Parameters
        ----------
        start : Tuple[float, float]
            One end of the edge
        end : Tuple[float, float]
            The other end of the edge

        Returns
        -------
        bool
            whether the edge falls within E_free
        """
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dist = np.sqrt(dx**2 + dy**2)
        ite = int(dist / 0.01)  # Check every 0.01u along the edge
        for i in range(ite + 1):
            if i == 0:
                t = 0 
            else:
                t = i / ite
            x = start[0] + t*dx
            y = start[1] + t*dy
            if self.env.check_collision(x, y):
                return False
        return True

    def construct_roadmap(self):
        """
        Constructs roadmap of adjacency graph format using KD tree.
        """
        for i, node in enumerate(self.nodes):
            distances, node_indices = self.kd_tree.query(node, k=self.k_neighbors+1)
            self.graph[i] = []
            for j, dist in zip(node_indices[1:], distances[1:]):  # First node is skipped as it's the node itself
                if dist <= self.max_edge_length:
                    if self.check_edge(node, self.nodes[j]):
                        self.graph[i].append(j)

    def heuristic_a_star(self, curr, nex):
        """
        Consistent heuristic for A* search i.e. Euclidean distance between given nodes.
        Consistent as Euclidean distance is the optimal solution to a simplified problem,
            where no obstacles exist.

        Parameters
        ----------
        curr : Tuple[float, float]
            Current node
        nex : Tuple[float, float]
            Next node

        Returns
        -------
        float
            heurisic value
        """
        return np.linalg.norm(np.array(self.nodes[curr]) - np.array(self.nodes[nex]))

    def heuristic_ucs(self, curr, nex):
        """
        Heuristic for the use of Uniform Cost Search (if needed), since A* with heuristic = 0
            is equivalent to UCS.

        Parameters
        ----------
        curr : Tuple[float, float]
            Current node
        nex : Tuple[float, float]
            Next node

        Returns
        -------
        float
            0
        """
        return 0

    def a_star_search(self, start, goal, heuristic):
        """
        Uses A* search to find optimal path from start to goal.

        Parameters
        ----------
        start : Tuple[float, float]
            Start node
        goal : Tuple[float, float]
            Goal node
        heuristic : Callable[[Tuple[float, float], Tuple[float, float]], float]
            Heuristic function

        Returns
        -------
        List[Tuple[float, float]]
            optimal path i.e. sequence of nodes to be traversed
        """
        def edge_cost(curr, nex):
            return self.heuristic_a_star(curr, nex)

        frontier = []
        prev_node = {}
        g_n = {start: 0}
        f_n = {start: heuristic(start, goal)}
        heapq.heappush(frontier, (0, start))

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal:
                path = []
                while current in prev_node:
                    path.append(current)
                    current = prev_node[current] # trace the path back to start
                path.append(start)
                return path[::-1]

            for neighbor in self.graph[current]:
                """
                A* search - to minimize the following:
                f_n = g_n + h_n
                  - g_n: actl cost from start to n
                  - h_n: heuristic from n to goal
                """
                tentative_g_n = g_n[current] + edge_cost(current, neighbor)
                if neighbor not in g_n or tentative_g_n < g_n[neighbor]:
                    prev_node[neighbor] = current
                    g_n[neighbor] = tentative_g_n
                    f_n[neighbor] = g_n[neighbor] + heuristic(neighbor, goal)
                    heapq.heappush(frontier, (f_n[neighbor], neighbor))

        return None

    def add_query_node(self, point):
        """
        Adds a given node and connects it to existing E_free.

        Parameters
        ----------
        point : Tuple[float, float]
            Node to be added

        Returns
        -------
        int
            Unique index representing this node
        """
        if self.env.check_collision(point[0], point[1]):
            return None
        distances, node_indices = self.kd_tree.query(point, k=self.k_neighbors)
        node_id = len(self.nodes)
        self.nodes.append(point)
        self.graph[node_id] = []
        for j, dist in zip(node_indices, distances):
            if dist <= self.max_edge_length:
                if self.check_edge(point, self.nodes[j]):
                    self.graph[node_id].append(j)
                    self.graph[j].append(node_id) # creates undirected edge betw. node and neighbor
        return node_id

    def find_path(self, start, goal, heuristic):
        """
        Adds start and goal nodes to E_free and searches for optimal path.

        Parameters
        ----------
        start : Tuple[float, float]
            Start node
        goal : Tuple[float, float]
            Goal node
        heuristic : Callable[[Tuple[float, float], Tuple[float, float]], float]
            Heuristic function

        Returns
        -------
        List[Tuple[float, float]]
            optimal path i.e. sequence of nodes to be traversed
        """
        start_node = self.add_query_node(start)
        goal_node = self.add_query_node(goal)
        
        if start_node is None or goal_node is None:
            return None

        path = self.a_star_search(start_node, goal_node, heuristic)
        if path:
            return [self.nodes[node] for node in path]
        return None

    def do_path_shortcutting(self, path, max_reps=20):
        """
        Does path shortcutting by "adding" new, shorter edges.

        Parameters
        ----------
        path : List[Tuple[float, float]]
            Existing path
        max_reps : int
            Maximum iterations of edge cutting

        Returns
        -------
        List[Tuple[float, float]]
            shorter path
        """
        if path is None or len(path) < 3:
            return path

        for _ in range(max_reps):
            a, b = sorted(random.sample(range(len(path)), 2))
            if self.check_edge(path[a], path[b]):
                path = path[:a+1] + path[b:]

        return path

    def find_path_cost(self, path):
        """
        Calculates total path length/cost of given path.

        Parameters
        ----------
        path : List[Tuple[float, float]]
            Existing path

        Returns
        -------
        float
            total path cost
        """
        if path is None:
            return path
        def edge_cost(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))
        return sum(edge_cost(path[i], path[i+1]) for i in range(len(path) - 1))

    def plot_roadmap(self):
        """
        Plots all nodes and edges on the environment.
        """
        self.env.plot()
        for node, neighbors in self.graph.items():
            x1, y1 = self.nodes[node]
            for neighbor in neighbors:
                x2, y2 = self.nodes[neighbor]
                pl.plot([x1, x2], [y1, y2], 'g-', alpha=0.15)
        pl.plot(*zip(*self.nodes), 'ro', markersize=2)

    def trace_path(self, path):
        """
        Plots a trace of the given path on the environment.

        Parameters
        ----------
        path : List[Tuple[float, float]]
            Path to be traced
        """
        if path:
            path_coords = np.array(path)
            pl.plot(path_coords[:, 0], path_coords[:, 1], 'b-', linewidth=2)
