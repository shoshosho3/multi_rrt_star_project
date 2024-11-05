import numpy as np


def coords_dist(coords1, coords2):
    if coords1 is None or coords2 is None:
        return float('inf')
    return np.linalg.norm(coords1 - coords2)


def is_tree_pair_valid(tree1, tree2):
    return tree1 != tree2 and \
        not (tree1.tree_type == 'robot' and tree2.tree_type == 'robot')


def init_trees(robot_sources, goal_sources):
    trees, robot_trees, goal_trees = [], [], []
    count = 0
    for source in robot_sources:
        tree = Tree('robot', count, source)
        robot_trees.append(tree)
        trees.append(tree)
        count += 1
    for source in goal_sources:
        tree = Tree('goal', count, source)
        goal_trees.append(tree)
        trees.append(tree)
        count += 1
    return trees, robot_trees, goal_trees


class Node:
    def __init__(self, tree, coordinates, parent=None, cost=0, children=None):
        self.tree = tree
        self.coordinates = coordinates
        self.parent = parent
        self.cost = cost
        self.children = children if children is not None else []

    def find_nearest(self, coordinates):
        best_node, best_dist = self, coords_dist(self.coordinates, coordinates)
        for child in self.children:
            child_node, child_dist = child.find_nearest(coordinates)
            if best_dist > child_dist:
                best_node, best_dist = child_node, child_dist
        return best_node, best_dist

    def path_to_root(self):
        node = self
        path = [node]
        while node.parent is not None:
            node = node.parent
            path.append(node)
        return path

    def get_neighbors(self, coordinates, radius):
        neighbors = []
        self_dist = coords_dist(self.coordinates, coordinates)
        if self_dist <= radius:
            neighbors.append(self)
        for child in self.children:
            child_neighbors = child.get_neighbors(coordinates, radius)
            neighbors.extend(child_neighbors)
        return neighbors

    def update_cost(self, cost_diff):
        self.cost += cost_diff
        for child in self.children:
            child.update_cost(cost_diff)

    def check_connection(self, rrt_solver):
        for connection in rrt_solver.tree_connections.values():
            if connection.node1 == self or connection.node2 == self:
                return True
        for child in self.children:
            if child.check_connection(rrt_solver):
                return True
        return False

    def branch_and_bound(self, rrt_solver):
        for other_tree in rrt_solver.trees:
            if not is_tree_pair_valid(other_tree, self.tree):
                continue
            c_best = rrt_solver.get_c_best(self.tree, other_tree)
            cost_bound = self.cost + other_tree.cost_to_go(self.coordinates)
            if cost_bound <= c_best:
                for child in self.children:
                    child.branch_and_bound(rrt_solver)
                return
        if self.check_connection(rrt_solver):
            return
        self.prune()

    def prune(self):
        self.tree = None
        self.coordinates = None
        if self.parent is not None:
            self.parent.children.remove(self)
        self.parent = None
        for child in self.children:
            child.prune()
        self.children = None


class Tree:
    def __init__(self, tree_type, tree_id, root_source):
        self.tree_type = tree_type  # 'robot'/'goal'
        self.tree_id = tree_id
        self.root = Node(self, root_source)

    def find_nearest(self, coordinates):
        best_node, best_dist = self.root.find_nearest(coordinates)
        return best_node

    def get_neighbors(self, coordinates, radius):
        return self.root.get_neighbors(coordinates, radius)

    def cost_to_go(self, coords):
        return coords_dist(coords, self.root.coordinates)

    def branch_and_bound(self, rrt_solver):
        self.root.branch_and_bound(rrt_solver)


class Connection:
    def __init__(self, node1, node2, check_collision, between_trees=False):
        self.node1 = node1
        self.node2 = node2
        self.check_collision = check_collision
        self.length = coords_dist(node1.coordinates, node2.coordinates)
        self.between_trees = between_trees
        self.is_valid = None

    def cost(self):
        if not self.between_trees:
            return self.node1.cost + self.length
        return self.node1.cost + self.length + self.node2.cost

    def get_is_valid(self):
        if self.is_valid is None:
            self.is_valid = not self.check_collision(self.node1.coordinates, self.node2.coordinates)
        return self.is_valid
