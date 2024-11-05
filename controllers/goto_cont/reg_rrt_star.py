import numpy as np
import random
from consts import *
import unit_transformations as ut


class Node:
    """
    Represents a node in the RRT* algorithm.

    Attributes:
        position (tuple): The (x, y) coordinates of the node.
        parent (Node): The parent node in the tree, used to trace back the path.
        cost (float): The cumulative cost to reach this node from the start node.
    """

    def __init__(self, position):
        self.position = position
        self.parent = None
        self.cost = 0


def euclidean_distance(point1, point2):
    """
    Calculates the Euclidean distance between two points.

    Args:
        point1 (tuple): The (x, y) coordinates of the first point.
        point2 (tuple): The (x, y) coordinates of the second point.

    Returns:
        float: The Euclidean distance between the two points.
    """
    return np.linalg.norm(np.array(point1) - np.array(point2))


def check_collision(coords1, coords2, obstacles):
    """
    Checks for a collision along a line from coords1 to coords2 using Bresenham's Line Algorithm.

    Args:
        coords1 (tuple): The starting (x, y) coordinates.
        coords2 (tuple): The ending (x, y) coordinates.
        obstacles (numpy.ndarray): A 2D boolean array where True indicates an obstacle.

    Returns:
        bool: True if there is a collision with an obstacle, False otherwise.
    """
    x1, y1 = coords1
    x2, y2 = coords2
    # Round coordinates and initialize Bresenham's algorithm
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
        # Check if out of bounds or an obstacle
        if x1 < 0 or x1 >= FLOOR_LENGTH or y1 < 0 or y1 >= FLOOR_LENGTH:
            return True
        if obstacles[x1][y1] == HAS_OBSTACLE:
            return True

        if x1 == x2 and y1 == y2:
            break

        # Update the error term and coordinates for Bresenham's line traversal
        e2 = 2 * err
        if e2 >= -dy:
            err -= dy
            x1 += sx
        if e2 <= dx:
            err += dx
            y1 += sy

    return False


def nearest_node(tree, point):
    """
    Finds the node in the tree closest to a given point.

    Args:
        tree (list of Node): The current set of nodes in the tree.
        point (tuple): The (x, y) coordinates to find the nearest node to.

    Returns:
        Node: The closest node in the tree to the given point.
    """
    return min(tree, key=lambda node: euclidean_distance(node.position, point))


def steer(from_point, to_point, max_step_size):
    """
    Moves from 'from_point' towards 'to_point' by a distance up to 'max_step_size'.

    Args:
        from_point (tuple): The starting (x, y) coordinates.
        to_point (tuple): The target (x, y) coordinates.
        max_step_size (float): The maximum distance to move from 'from_point' towards 'to_point'.

    Returns:
        tuple: The new (x, y) coordinates after moving from 'from_point' towards 'to_point'.
    """
    direction = np.array(to_point) - np.array(from_point)
    distance = np.linalg.norm(direction)
    if distance > max_step_size:
        direction = direction / distance * max_step_size
    new_point = np.array(from_point) + direction
    return tuple(map(int, new_point))


def get_neighbors(tree, new_node, radius):
    """
    Finds nodes within a given radius of a new node.

    Args:
        tree (list of Node): The current set of nodes in the tree.
        new_node (Node): The new node to find neighbors around.
        radius (float): The radius within which to find neighboring nodes.

    Returns:
        list of Node: The list of neighboring nodes within the specified radius.
    """
    return [node for node in tree if euclidean_distance(node.position, new_node.position) < radius]


def path_to_goal(goal_node):
    """
    Traces back from the goal node to construct the path from start to goal.

    Args:
        goal_node (Node): The node representing the goal.

    Returns:
        list of tuple: The ordered list of (x, y) positions from start to goal.
    """
    path = []
    node = goal_node
    while node is not None:
        path.append(node.position)
        node = node.parent
    return path[::-1]


def run_rrt_star(start, goal, obstacle_matrix, max_iterations=MAX_ITERATIONS_REG_RRT,
                 max_step_size=STEP_SIZE, radius=RADIUS):
    """
    Runs the RRT* algorithm from a start point to a goal on a grid with obstacles.

    Args:
        start (tuple): The starting (x, y) coordinates in GPS units.
        goal (tuple): The goal (x, y) coordinates in GPS units.
        obstacle_matrix (numpy.ndarray): A 2D boolean array where True indicates an obstacle.
        max_iterations (int): The maximum number of iterations to run the algorithm.
        max_step_size (float): The maximum step size for each move in the tree.
        radius (float): The radius within which to consider neighbors for rewiring.

    Returns:
        list of tuple or None: The path from start to goal as a list of GPS coordinates if a path is found, otherwise None.
    """
    final_path = None
    start = ut.gps_to_floor(start)
    goal = ut.gps_to_floor(goal)
    start_node = Node(start)
    goal_node = Node(goal)
    tree = [start_node]
    best_cost = float('inf')

    i = 0

    while i < max_iterations or (final_path is None and i < max_iterations * 2):
        i += 1
        # Sample a random point, biased towards the goal
        if random.random() < RANDOM_POINT_THRESHOLD:
            random_point = goal
        else:
            random_point = (
                random.randint(0, FLOOR_LENGTH), random.randint(0, FLOOR_LENGTH - 1))

        # Find the nearest node in the tree and steer towards the random point
        nearest = nearest_node(tree, random_point)
        new_position = steer(nearest.position, random_point, max_step_size)

        # Skip if new position is out of bounds or in an obstacle
        if (new_position[0] < 0 or new_position[0] >= FLOOR_LENGTH or
                new_position[1] < 0 or new_position[1] >= FLOOR_LENGTH):
            continue
        if obstacle_matrix[new_position[0]][new_position[1]]:
            continue

        # Check for collision-free path and add new node if valid
        if not check_collision(nearest.position, new_position, obstacle_matrix):
            new_node = Node(new_position)
            new_node.parent = nearest
            new_node.cost = nearest.cost + euclidean_distance(nearest.position, new_position)

            # Rewire if a cheaper path through new_node is found
            neighbors = get_neighbors(tree, new_node, radius)
            for neighbor in neighbors:
                cost = neighbor.cost + euclidean_distance(neighbor.position, new_position)
                if cost < new_node.cost and not check_collision(neighbor.position, new_position, obstacle_matrix):
                    new_node.parent = neighbor
                    new_node.cost = cost

            # Update neighbors if rewiring through new_node offers a cost reduction
            for neighbor in neighbors:
                cost = new_node.cost + euclidean_distance(new_node.position, neighbor.position)
                if cost < neighbor.cost and not check_collision(new_node.position, neighbor.position, obstacle_matrix):
                    neighbor.parent = new_node
                    neighbor.cost = cost

            tree.append(new_node)

            # Check if goal is reached and update best path if found
            if (euclidean_distance(new_position, goal) < CHARGING_DISTANCE / GPS_LENGTH * FLOOR_LENGTH
                    and new_node.cost < best_cost):
                best_cost = new_node.cost
                final_path = []
                get_path = path_to_goal(new_node)
                for loc in get_path:
                    final_path.append(ut.floor_to_gps(loc))

    return final_path
