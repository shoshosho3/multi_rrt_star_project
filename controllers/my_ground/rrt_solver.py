import numpy as np
import matplotlib.pyplot as plt
from consts import *
from tree_classes import Node, is_tree_pair_valid, coords_dist, init_trees, Connection
from sampling_classes import InformedSampler
from segment import SegmentDistance


def organize_possible_connections(new_node, neighbors, check_collision, between_trees=False):
    """
    Organize the possible connections between a new node and its neighbors.
    Sorted by connection cost.

    new_node: A node to be added in the rrt* algorithm
    neighbors: All nodes in the tree that are at distance 'radius' at most.
    check_collision: # obstacle collision checking function
    between_trees: If the tree of neighbors is different from the tree of new_node
    """
    possible_connections = []
    for neighbor in neighbors:
        connection = Connection(neighbor, new_node, check_collision, between_trees)
        possible_connections.append(connection)
    possible_connections.sort(key=lambda x: x.cost())
    return possible_connections


def new_choose_parent(new_node, possible_connections, target_tree, c_best):
    """
    Choose a parent to new_node from possible_connections.

    possible_connections: connections of parent candidates of new_node.
    from the same tree of new_node (different from target_tree).
    target_tree: The tree new_node tries to connect to.
    c_best: The current best cost to connect target_tree to the tree of new_node.
    """
    for connection in possible_connections:
        # if the connection can result in improvement and is valid
        if connection.get_is_valid():
            new_node.cost = connection.cost()
            new_node.parent = connection.node1
            connection.node1.children.append(new_node)
            return True
    return False


def new_make_children(new_node, possible_connections):
    for connection in possible_connections:
        neighbor = connection.node1
        # If rewiring will reduce cost and is valid
        if new_node.cost + connection.length < neighbor.cost:
            if connection.get_is_valid():
                # rewire
                old_parent = neighbor.parent
                if old_parent is not None:
                    old_parent.children.remove(neighbor)
                neighbor.parent = new_node
                cost_diff = new_node.cost + connection.length - neighbor.cost
                neighbor.update_cost(cost_diff)
                new_node.children.append(neighbor)


class NewRRTSolver:
    def __init__(self, robot_sources, goal_sources, bounds, p_task_space,
                 steer_step_size, is_coords_valid, obstacles,
                 p_vertex_contraction, min_drive_speed, max_drive_speed,
                 max_turn_time, collision_distance):
        """
        robot_sources: a list of robot coordinates
        goal_sources: a list of goal coordinates
        bounds: the bound to the space. [[low_bound1, low_bound2], [high bound1, high bound2]]
        p_task_space: probability of sampling point using a heuristic for connecting two trees
        steer_step_size: The max step size in rrt*
        is_coords_valid: function checking if coords not collide with obstacles.
        collision_step_size: step size when checking for obstacle collision between point.
        p_vertex_contraction: probability of vertex contraction in rrt*
        min_drive_speed: The mininal drive speed of the robot.
        max_drive_speed: The maximal drive speed of the robot.
        max_turn_time: The maximal time a robot will spend in a point (to turn or other).
        collision_distance: the maximal distance between two robots to be considered collision.
        """
        self.trees, self.robot_trees, self.goal_trees = init_trees(robot_sources, goal_sources)
        self.bounds = bounds
        self.p_task_space = p_task_space
        self.steer_step_size = steer_step_size
        self.tree_connections = {}
        self.is_coords_valid = is_coords_valid
        self.p_vertex_contraction = p_vertex_contraction
        self.min_drive_speed = min_drive_speed
        self.max_drive_speed = max_drive_speed
        self.max_turn_time = max_turn_time
        self.collision_distance = collision_distance
        self.obstacles = obstacles

    def expend_tree(self, expended_tree, target_tree):
        """
        Add one node to expended_tree in the direction of target_tree
        """
        sampled_point = self.sample_point(expended_tree, target_tree)
        nearest_node = expended_tree.find_nearest(sampled_point)
        new_coords = self.steer(nearest_node.coordinates, sampled_point)
        if not self.is_coords_valid(new_coords, self.obstacles):
            return
        new_node = Node(expended_tree, new_coords)
        neighbors = expended_tree.get_neighbors(new_coords, self.steer_step_size * 2)

        possible_connections = organize_possible_connections(new_node, neighbors, self.check_collision)
        c_best = self.get_c_best(expended_tree, target_tree)
        parent_found = new_choose_parent(new_node, possible_connections, target_tree, c_best)
        if not parent_found:
            return
        new_make_children(new_node, possible_connections)

        self.try_connect_graphs(expended_tree, new_node)
        self.random_vertex_contraction(new_node)
        self.branch_and_bound(expended_tree)

    def random_expend_tree(self):
        """
        Randomly choose a tree to expend and a target tree.
        """
        expended_tree, target_tree = self.smart_sample_trees()
        self.expend_tree(expended_tree, target_tree)

    def get_trees_connection(self, tree1, tree2):
        """Get the connection between two trees"""
        tree_ids = tuple(sorted((tree1.tree_id, tree2.tree_id)))
        return self.tree_connections.get(tree_ids)

    def get_c_best(self, tree1, tree2):
        """Get the current best distance between tree1 and tree2"""
        connection = self.get_trees_connection(tree1, tree2)
        if connection is None:
            return float('inf')
        return connection.cost()

    def set_c_best(self, tree1, tree2, c_best, tree_connection):
        """Set the current best distance between tree1 and tree2"""
        tree_ids = tuple(sorted((tree1.tree_id, tree2.tree_id)))
        self.tree_connections[tree_ids] = tree_connection

    def try_connect_graphs(self, expended_tree, new_node):
        """Attempt to connect new_node to valid trees"""
        for connect_tree in self.trees:
            if not is_tree_pair_valid(expended_tree, connect_tree):
                continue
            c_best = self.get_c_best(expended_tree, connect_tree)
            connection = self.connect_graphs(connect_tree, new_node, c_best)
            if connection is not None:
                self.set_c_best(expended_tree, connect_tree, connection.cost(), connection)

    def connect_graphs(self, target_tree, new_node, c_best):
        """Attempt to connect new_nod to a specific tree"""
        target_nearest = target_tree.find_nearest(new_node.coordinates)
        target_neighbors = target_tree.get_neighbors(new_node.coordinates, self.steer_step_size * 2)
        possible_connections = organize_possible_connections(new_node, target_neighbors, self.check_collision,
                                                             between_trees=True)

        for connection in possible_connections:
            if connection.cost() < c_best and connection.get_is_valid():
                return connection
        return None

    def branch_and_bound(self, expended_tree):
        """Prune nodes that are unlikely to result in cost reduction"""
        if self.is_tree_connected(expended_tree):
            expended_tree.branch_and_bound(self)

    def sample_point(self, expended_tree, target_tree):
        p = np.random.rand()
        if p < self.p_task_space and self.get_c_best(expended_tree, target_tree) != float('inf'):
            return self.sample_task_space(expended_tree, target_tree)
        return self.basic_sample(expended_tree, target_tree)

    def random_vertex_contraction(self, node):
        """Attempt to contract the path from node to its root"""
        p = np.random.rand()
        if p > self.p_vertex_contraction:
            return
        path = node.path_to_root()
        if len(path) < 3:
            return
        descendant_idx = np.random.randint(0, len(path) - 2)
        ancestor_idx = np.random.randint(descendant_idx + 2, len(path))
        descendant_node = path[descendant_idx]
        ancestor_node = path[ancestor_idx]

        if not self.check_collision(descendant_node.coordinates, ancestor_node.coordinates):
            cost_diff = ancestor_node.cost + coords_dist(descendant_node.coordinates,
                                                         ancestor_node.coordinates) - descendant_node.cost
            descendant_node.update_cost(cost_diff)
            old_parent = descendant_node.parent
            if old_parent is not None:
                old_parent.children.remove(descendant_node)
            ancestor_node.children.append(descendant_node)
            descendant_node.parent = ancestor_node

    def sample_task_space(self, expended_tree, target_tree):
        """Greedy sampling"""
        closest_node = expended_tree.find_nearest(target_tree.root.coordinates)
        dist = coords_dist(closest_node.coordinates, target_tree.root.coordinates)
        if dist < self.steer_step_size:
            return self.basic_sample(expended_tree, target_tree)
        sampler = InformedSampler(closest_node.coordinates, target_tree.root.coordinates)
        # Implement task-space sampling logic
        return sampler.sample(1.2 * dist)

    def basic_sample(self, expended_tree, target_tree):
        c_best = self.get_c_best(expended_tree, target_tree)
        if c_best == float('inf'):  # if trees aren't connected
            return np.random.uniform(self.bounds[0], self.bounds[1])
        x_goal = target_tree.root.coordinates
        x_start = expended_tree.root.coordinates
        sampler = InformedSampler(x_goal, x_start)
        if c_best <= sampler.cmin:  # if optimal cost
            return np.random.uniform(self.bounds[0], self.bounds[1])

        return sampler.sample(c_best)

    def steer(self, coord1, coord2):
        direction = coord2 - coord1
        distance = np.linalg.norm(direction)
        if distance <= self.steer_step_size:
            return coord2
        try:
            direction /= distance
        except Warning:
            print(direction)
            print(distance)
        return coord1 + direction * self.steer_step_size

    def sample_trees(self):
        """Sample a tree pair to expand"""
        expended_tree, target_tree = np.random.choice(self.trees, 2)
        while not is_tree_pair_valid(expended_tree, target_tree):
            expended_tree, target_tree = np.random.choice(self.trees, 2)
        return expended_tree, target_tree

    def smart_sample_trees(self):
        """Sample a tree pair to expand. favor unconnected pairs"""
        expended_tree = np.random.choice(self.trees)
        unconnected_trees = self.unconnect_trees(expended_tree)
        if len(unconnected_trees) == 0:
            target_tree = np.random.choice(expended_tree.valid_trees)
            return expended_tree, target_tree
        target_tree = np.random.choice(unconnected_trees)
        return expended_tree, target_tree

    def unconnect_trees(self, tree):
        """
        Find all valid trees that are not connected to tree.
        """
        unconnected = []
        for valid_tree in tree.valid_trees:
            if self.get_c_best(tree, valid_tree) == float('inf'):
                unconnected.append(valid_tree)
        return unconnected

    def is_tree_connected(self, tree):
        """
        Check if all valid tree couples of 'tree' are connected.
        """
        for other_tree in self.trees:
            if not is_tree_pair_valid(other_tree, tree):
                continue
            c_best = self.get_c_best(tree, other_tree)
            if c_best == float('inf'):
                return False
        return True

    def check_collision(self, coords1, coords2):
        x1, y1 = coords1
        x2, y2 = coords2
        # Bresenham's Line Algorithm to walk along the grid cells
        x1 = round(x1)
        y1 = round(y1)
        x2 = round(x2)
        y2 = round(y2)
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while True:
            if x1 < 0 or x1 >= FLOOR_LENGTH or y1 < 0 or y1 >= FLOOR_LENGTH:
                return True
            # Check if the current cell has an obstacle (1)
            if self.obstacles[x1][y1] == HAS_OBSTACLE:
                return True

            # If we have reached the end point, break the loop
            if x1 == x2 and y1 == y2:
                break

            # Update the error and coordinates
            e2 = 2 * err
            if e2 >= -dy:
                err -= dy
                x1 += sx
            if e2 <= dx:
                err += dx
                y1 += sy

        return False

    def are_trees_connected(self):
        """
        Check if all valid tree couples are connected.
        """
        for i, tree in enumerate(self.trees):
            if not self.is_tree_connected(tree):
                return False
        return True

    def extract_connection_path(self, start_tree, goal_tree):
        """
        Extract the path from start_tree's root to goal_tree's root.
        """
        connection = self.get_trees_connection(start_tree, goal_tree)
        start_connection = connection.node1 if connection.node1.tree == start_tree else connection.node2
        goal_connection = connection.node1 if connection.node1.tree == goal_tree else connection.node2
        start_part = start_connection.path_to_root()
        goal_part = goal_connection.path_to_root()
        return start_part[::-1] + goal_part

    def line_segments_intersect(self, start1, end1, start2, end2):
        """
        Compute if two line segments intersect in space.
        """
        distance = SegmentDistance.compute_distance(start1, end1, start2, end2).distance
        return distance < self.collision_distance

    def is_collision_path_one_side(self, start1node, end1node, start1low, end1up,
                                   concat_path2, low_bound2, idx2):
        """
        Check if one connection intersect a path in time and space.
        Used in is_collision_path

        idx2: Starting index to check from in path2
        """
        if idx2 >= len(concat_path2) - 1:
            return False
        start2node, end2node = concat_path2[idx2], concat_path2[idx2 + 1]
        start2low = low_bound2[idx2]
        while idx2 < len(concat_path2) - 1 and end1up > start2low:
            start2node, end2node = concat_path2[idx2], concat_path2[idx2 + 1]
            start2low = low_bound2[idx2]
            if self.line_segments_intersect(start1node.coordinates, end1node.coordinates,
                                            start2node.coordinates, end2node.coordinates):
                return True
            idx2 += 1
        return False

    def is_collision_path(self, concat_path1, low_bound1, up_bound1,
                          concat_path2, low_bound2, up_bound2):
        """
        Check if two paths are colliding in time and space.
        """
        idx1, idx2 = 0, 0
        while idx1 < len(concat_path1) - 1 and idx2 < len(concat_path2) - 1:
            start1node, start2node = concat_path1[idx1], concat_path2[idx2]
            start1low, start2low = low_bound1[idx1], low_bound2[idx2]
            start1up, start2up = up_bound1[idx1], up_bound2[idx2]

            end1node, end2node = concat_path1[idx1 + 1], concat_path2[idx2 + 1]
            end1low, end2low = low_bound1[idx1 + 1], low_bound2[idx2 + 1]
            end1up, end2up = up_bound1[idx1 + 1], up_bound2[idx2 + 1]
            # if no intersection in time
            if start1low > end2up:
                idx2 += 1
                continue
            if start2low > end1up:
                idx1 += 1
                continue
            # if intersect in time
            if self.line_segments_intersect(start1node.coordinates, end1node.coordinates,
                                            start2node.coordinates, end2node.coordinates):
                return True
            if self.is_collision_path_one_side(start1node, end1node, start1low, end1up,
                                               concat_path2, low_bound2, idx2 + 1):
                return True
            if self.is_collision_path_one_side(start2node, end2node, start2low, end2up,
                                               concat_path1, low_bound1, idx1 + 1):
                return True
            idx1 += 1
            idx2 += 1
        return False

    def concat_path(self, path, robot_idx):
        """
        Concatinate a path made out of sub-paths to a single path.
        Compute time bounds to entering and exiting every node.
        """
        prev_node = self.robot_trees[robot_idx].root
        concat_path_ = [prev_node, prev_node]
        low_bound = [0, 0]
        up_bound = [0, self.max_turn_time]
        low = 0
        up = self.max_turn_time

        for sub_path in path:
            for node in sub_path[1:]:
                dist = coords_dist(prev_node.coordinates, node.coordinates)
                low += dist / self.max_drive_speed
                up += dist / self.min_drive_speed
                concat_path_.append(node)
                low_bound.append(low)
                up_bound.append(up)
                up += self.max_turn_time
                concat_path_.append(node)
                low_bound.append(low)
                up_bound.append(up)
                prev_node = node
        low_bound[-1] = float('inf')
        up_bound[-1] = float('inf')
        return concat_path_, low_bound, up_bound

    def concat_paths(self, paths):
        """
        For every robot, concatinate the given path made out of sub-paths to a single path.
        Compute time bounds to entering and exiting every node.
        """
        concat_paths_, low_bounds, up_bounds = [], [], []
        for idx in range(len(self.robot_trees)):
            concat_path_, low_bound, up_bound = self.concat_path(paths[idx], idx)
            concat_paths_.append(concat_path_)
            low_bounds.append(low_bound)
            up_bounds.append(up_bound)
        return concat_paths_, low_bounds, up_bounds

    def get_static_c_best(self):
        """
        Make a static version of c_best
        """
        static_c_best = {}
        for tree1 in self.trees:
            for tree2 in self.trees:
                if not is_tree_pair_valid(tree1, tree2):
                    continue
                c_best = self.get_c_best(tree1, tree2)
                static_c_best[(tree1.tree_id, tree2.tree_id)] = c_best
                static_c_best[(tree2.tree_id, tree1.tree_id)] = c_best
        return static_c_best

    def random_allocate_goals(self, num_tries, check_collision=True):
        """
        Randomly sample num_tries goal allocations and return the best one.
        """
        best_allocation, best_cost = None, float('inf')
        static_c_best = self.get_static_c_best()
        for _ in range(num_tries):
            allocation, allocation_cost = self.sample_allocation(static_c_best)
            if allocation_cost < best_cost and (not check_collision or not self.is_collision_allocation(allocation)):
                best_allocation, best_cost = allocation, allocation_cost
        return best_allocation, best_cost

    def extract_alloaction_paths(self, robots_allocation):
        """
        For every robot, parse the allocation to a path of sub-paths.
        """
        allocation_paths = []
        for idx in range(len(self.robot_trees)):
            allocation = robots_allocation[idx]
            if len(allocation) == 0:
                allocation_paths.append([])
                continue
            start_tree = self.robot_trees[idx]
            end_tree = self.goal_trees[allocation[0]]
            allocation_path = [self.extract_connection_path(start_tree, end_tree)]
            for i in range(1, len(allocation)):
                start_tree = self.goal_trees[allocation[i - 1]]
                end_tree = self.goal_trees[allocation[i]]
                allocation_path.append(self.extract_connection_path(start_tree, end_tree))
            allocation_paths.append(allocation_path)
        return allocation_paths

    def is_collision_allocation(self, allocation):
        """
        Check if the given allocation is colliding with itself.
        """
        # extract paths
        allocation_paths = self.extract_alloaction_paths(allocation)
        concat_paths_, low_bounds, up_bounds = self.concat_paths(allocation_paths)
        for idx1 in range(len(self.robot_trees)):
            for idx2 in range(idx1 + 1, len(self.robot_trees)):
                if self.is_collision_path(concat_paths_[idx1], low_bounds[idx1], up_bounds[idx1],
                                          concat_paths_[idx2], low_bounds[idx2], up_bounds[idx2]):
                    return True
        return False

    def sample_allocation(self, static_c_best):
        """
        Greedily sample an allocation of goals to robots with order of goals.
        Returns the allocation and its cost.
        """
        allocation = [[] for i in range(len(self.robot_trees))]
        allocation_costs = np.zeros(len(allocation))
        goal_probs = np.ones(len(self.goal_trees))
        goal_idxes = np.array(range(len(self.goal_trees)))
        robot_idxes = np.array(range(len(self.robot_trees)))
        for i in range(len(self.goal_trees)):
            # sample goal
            sampled_goal_idx = np.random.choice(goal_idxes, p=goal_probs / np.sum(goal_probs))
            sampled_goal_id = self.goal_trees[sampled_goal_idx].tree_id
            goal_probs[sampled_goal_idx] = 0
            # calc cost of choosing every robot
            robot_dists = np.zeros(len(self.robot_trees))
            for robot_idx in robot_idxes:
                if len(allocation[robot_idx]) == 0:
                    sampled_robot_id = self.robot_trees[robot_idx].tree_id
                else:
                    sampled_robot_id = self.goal_trees[allocation[robot_idx][-1]].tree_id
                robot_dists[robot_idx] = static_c_best[(sampled_robot_id, sampled_goal_id)]
            # sample robot
            pseudo_prob1 = (allocation_costs + robot_dists) / np.sum(allocation_costs + robot_dists)
            pseudo_prob2 = 1 - pseudo_prob1
            pseudo_prob3 = (pseudo_prob2) / np.sum(pseudo_prob2)
            if len(robot_idxes) == 1:
                sampled_robot = 0
            else:
                sampled_robot = np.random.choice(robot_idxes, p=pseudo_prob3)
            allocation_costs[sampled_robot] += robot_dists[sampled_robot]
            allocation[sampled_robot].append(sampled_goal_idx)
        return allocation, np.max(allocation_costs)

    def plot_all_trees(self):
        """
        Plots all trees in the solver using matplotlib for a 2D space.
        Each tree is drawn in a unique color with lines connecting nodes.
        """
        plt.figure(figsize=(10, 10))
        colors = plt.cm.rainbow(np.linspace(0, 1, len(self.trees)))  # Generate unique colors for each tree

        for idx, tree in enumerate(self.trees):
            color = colors[idx]
            print(f"Plotting Tree ID: {tree.tree_id}, Type: {tree.tree_type}")
            x_root, y_root = tree.root.coordinates
            plt.scatter(x_root, y_root, color=color, s=80, marker='D')
            self.plot_tree_nodes(tree.root, color)

        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.title("Visualization of All Trees in 2D Space")
        # plt.show()

    def plot_tree_nodes(self, node, color):
        """
        Recursively plots each node and its children in a tree.

        Parameters:
        node (Node): The current node being plotted.
        color: Color for plotting the tree.
        """
        x, y = node.coordinates
        plt.scatter(x, y, color=color, s=50)  # Plot the node
        if node.parent:
            px, py = node.parent.coordinates
            plt.plot([px, x], [py, y], color=color, linewidth=1)  # Line from parent to node

        for child in node.children:
            self.plot_tree_nodes(child, color)

    def plot_connections(self):
        for connection in self.tree_connections.values():
            x1, y1 = connection.node1.coordinates
            x2, y2 = connection.node2.coordinates
            plt.plot([x1, x2], [y1, y2], 'black')
