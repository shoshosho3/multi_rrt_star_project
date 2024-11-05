import numpy as np
from consts import *


def coords_dist(coords1, coords2):
    """
    distance between two coordinates with a none check
    :param coords1: coordinates of the first point
    :param coords2: coordinates of the second point
    :return: distance between the two points if they are not None, else infinity
    """
    if coords1 is None or coords2 is None:
        return float('inf')
    return np.linalg.norm(coords1 - coords2)


def is_tree_pair_valid(tree1, tree2):
    """
    check if the pair of trees is valid - not the same tree and not both robots
    :param tree1: first tree
    :param tree2: second tree
    :return: true if the pair of trees is valid, false otherwise
    """
    return tree1 != tree2 and not (tree1.tree_type == ROBOT and tree2.tree_type == ROBOT)


def add_trees(init_id, sources, type, trees, type_trees):
    """
    add trees to the list of trees
    :param init_id: current tree id
    :param sources: sources coordinates
    :param type: robot or goal
    :param trees: all the trees list
    :param type_trees: list of the type of the tree
    """
    tree_id = init_id
    for source in sources:
        tree = Tree(type, tree_id, source)
        type_trees.append(tree)
        trees.append(tree)
        tree_id += 1


def init_trees(robot_sources, goal_sources):
    """
    initialize the trees and give them unique ids
    :param robot_sources: robot sources coordinates
    :param goal_sources: goal sources coordinates
    :return: the trees, the robot trees and the goal trees
    """
    trees, robot_trees, goal_trees = [], [], []
    add_trees(0, robot_sources, ROBOT, trees, robot_trees)
    add_trees(len(robot_sources), goal_sources, GOAL, trees, goal_trees)
    return trees, robot_trees, goal_trees


class Node:
    """
    class representing a node in the RRT tree
    """
    def __init__(self, tree, coordinates, parent=None, cost=0, children=None):
        """
        initialize the node
        :param tree: the tree the node belongs to
        :param coordinates: coordinates of the node
        :param parent: parent node
        :param cost: cost to reach the node
        :param children: children nodes
        """
        self.tree = tree
        self.coordinates = coordinates
        self.parent = parent
        self.cost = cost
        self.children = children if children is not None else []

    def find_nearest(self, coordinates):
        """
        find the nearest node from the node and its children to the given coordinates
        :param coordinates: coordinates to find the nearest node to
        :return: the nearest node and the distance to it
        """
        best_node, best_dist = self, coords_dist(self.coordinates, coordinates)
        for child in self.children:
            child_node, child_dist = child.find_nearest(coordinates)
            if best_dist > child_dist:
                best_node, best_dist = child_node, child_dist
        return best_node, best_dist

    def path_to_root(self):
        """
        get the path from the node to the root
        :return: the path from the node to the root
        """
        node = self
        path = [node]
        while node.parent is not None:
            node = node.parent
            path.append(node)
        return path

    def get_neighbors(self, coordinates, radius):
        """
        get the neighbors of the node and its children within a given radius
        :param coordinates: coordinates to find the neighbors to
        :param radius: radius to find the neighbors within
        :return: the neighbors of the node and its children within the given radius
        """
        neighbors = []
        self_dist = coords_dist(self.coordinates, coordinates)
        if self_dist <= radius:
            neighbors.append(self)
        for child in self.children:
            child_neighbors = child.get_neighbors(coordinates, radius)
            neighbors.extend(child_neighbors)
        return neighbors

    def update_cost(self, cost_diff):
        """
        update the cost of the node and its children
        :param cost_diff: cost difference to add to the cost
        """
        self.cost += cost_diff
        for child in self.children:
            child.update_cost(cost_diff)

    def check_connection(self, rrt_solver):
        """
        check if the node is connected to the other tree
        :param rrt_solver: rrt solver object
        :return: true if the node is connected to the other tree or its children are connected to the other tree,
         false otherwise
        """
        for connection in rrt_solver.tree_connections.values():
            if connection.node1 == self or connection.node2 == self:
                return True
        for child in self.children:
            if child.check_connection(rrt_solver):
                return True
        return False

    def branch_and_bound(self, rrt_solver):
        """
        branch and bound the node and its children
        :param rrt_solver: rrt solver object
        """

        for other_tree in rrt_solver.trees:

            # check if the pair of trees is valid
            if not is_tree_pair_valid(other_tree, self.tree):
                continue

            # calculate the cost bound and the c_best
            c_best = rrt_solver.get_c_best(self.tree, other_tree)
            cost_bound = self.cost + other_tree.cost_to_go(self.coordinates)

            # if the cost bound is less than or equal to the c_best, branch and bound the children
            if cost_bound <= c_best:
                for child in self.children:
                    child.branch_and_bound(rrt_solver)
                return

        # check if the node or its children are connected to the other tree and prune if not
        if self.check_connection(rrt_solver):
            return
        self.prune()

    def prune(self):
        """
        prune the node and its children from the tree
        """
        self.tree = None
        self.coordinates = None
        if self.parent is not None:
            self.parent.children.remove(self)
        self.parent = None
        for child in self.children:
            child.prune()
        self.children = None


class Tree:
    """
    class representing a tree in the RRT algorithm
    """
    def __init__(self, tree_type, tree_id, root_source):
        """
        initialize the tree
        :param tree_type: robot or goal
        :param tree_id: id of the tree
        :param root_source: the source coordinates of the root node
        """
        self.tree_type = tree_type
        self.tree_id = tree_id
        self.root = Node(self, root_source)

    def find_nearest(self, coordinates):
        """
        find the nearest node in the tree to the given coordinates
        :param coordinates: coordinates to find the nearest node to
        :return: nearest node to the given coordinates
        """
        best_node, best_dist = self.root.find_nearest(coordinates)
        return best_node

    def get_neighbors(self, coordinates, radius):
        """
        get the neighbors of the tree within a given radius of the given coordinates
        :param coordinates: coordinates to find the neighbors to
        :param radius: radius to find the neighbors within
        :return: the neighbors of the tree within the given radius of the given coordinates
        """
        return self.root.get_neighbors(coordinates, radius)

    def cost_to_go(self, coords):
        """
        calculate the cost to go from the root to the given coordinates
        :param coords: coordinates to calculate the cost to go to
        :return: the cost to go from the root to the given coordinates
        """
        return coords_dist(coords, self.root.coordinates)

    def branch_and_bound(self, rrt_solver):
        """
        branch and bound the tree
        :param rrt_solver: rrt solver object
        """
        self.root.branch_and_bound(rrt_solver)


class Connection:
    """
    class representing a connection between two rrt nodes, can be between trees
    """
    def __init__(self, node1, node2, check_collision, between_trees=False):
        """
        initialize the connection
        :param node1: first node
        :param node2: second node
        :param check_collision: check collision function
        :param between_trees: is the connection between trees
        """
        self.node1 = node1
        self.node2 = node2
        self.check_collision = check_collision
        self.length = coords_dist(node1.coordinates, node2.coordinates)
        self.between_trees = between_trees
        self.is_valid = None

    def cost(self):
        """
        calculate the cost of the connection - length of the connection plus the cost of the nodes
        :return: the cost of the connection
        """
        if not self.between_trees:
            return self.node1.cost + self.length
        return self.node1.cost + self.length + self.node2.cost

    def get_is_valid(self):
        """
        get the validity of the connection by checking if there is a collision between the nodes
        :return: true if the connection is valid, false otherwise
        """
        if self.is_valid is None:
            self.is_valid = not self.check_collision(self.node1.coordinates, self.node2.coordinates)
        return self.is_valid
