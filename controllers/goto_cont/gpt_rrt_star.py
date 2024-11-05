import numpy as np
import matplotlib.pyplot as plt
import random
import math
from itertools import permutations
from tqdm import tqdm
from typing import List, Tuple
from consts import *


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)


class RRTStar:
    def __init__(self, start, goal, width, height, obstacles, step_size=STEP_SIZE, max_iter=MAX_ITERATIONS,
                 radius=RADIUS):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.width = width
        self.height = height
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iter = max_iter
        self.radius = radius
        self.nodes = [self.start]

    def is_collision(self, x1, y1, x2, y2):
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

    def get_random_node(self):
        x = random.randint(0, self.width)
        y = random.randint(0, self.height)
        return Node(x, y)

    def nearest_node(self, random_node):
        return min(self.nodes, key=lambda node: distance(node, random_node))

    def steer(self, from_node, to_node):
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = int(from_node.x + self.step_size * math.cos(theta))
        if new_x < 0:
            new_x = 0
        if new_x >= self.width:
            new_x = self.width - 1
        new_y = int(from_node.y + self.step_size * math.sin(theta))
        if new_y < 0:
            new_y = 0
        if new_y >= self.height:
            new_y = self.height - 1
        return Node(new_x, new_y)

    def near_nodes(self, new_node):
        return [node for node in self.nodes if distance(node, new_node) <= self.radius]

    def plan_to_point(self, target):
        self.goal = Node(target[0], target[1])
        got_path = False
        best_path = None
        found = False
        for i in tqdm(range(self.max_iter)):
            if found and i > MAX_IF_FOUND:
                break
            rand_node = self.get_random_node()
            nearest = self.nearest_node(rand_node)
            new_node = self.steer(nearest, rand_node)

            if not self.is_collision(nearest.x, nearest.y, new_node.x, new_node.y):
                near_nodes = self.near_nodes(new_node)
                new_node.parent = nearest
                self.nodes.append(new_node)

                # Rewire
                for near_node in near_nodes:
                    if not self.is_collision(new_node.x, new_node.y, near_node.x, near_node.y):
                        if distance(self.start, new_node) + distance(new_node, near_node) < distance(
                                self.start, near_node):
                            near_node.parent = new_node

                if distance(new_node, self.goal) <= self.step_size:
                    found = True
                    self.goal.parent = new_node
                    if not got_path:
                        best_path = self.generate_path()
                        got_path = True
                    else:
                        new_path = self.generate_path()
                        if len(new_path) < len(best_path):
                            best_path = new_path
        return best_path

    def generate_path(self):
        path = []
        node = self.goal
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]  # Reverse path


def create_obstacle_matrix():
    obstacle_matrix = np.zeros((FLOOR_LENGTH, FLOOR_LENGTH))
    for x_position in range(WALL_X_LEFT - AVOID_DISTANCE, WALL_X_RIGHT + AVOID_DISTANCE):
        for y_position in (list(range(0, WALL_Y_UP + AVOID_DISTANCE)) +
                           list(range(WALL_Y_DOWN - AVOID_DISTANCE, FLOOR_LENGTH))):
            obstacle_matrix[x_position][y_position] = HAS_OBSTACLE
    return obstacle_matrix


def plot(dirt_locations, obstacles, path=None):
    plt.figure(figsize=(FIG_SIZE, FIG_SIZE))
    plt.xlim(0, FLOOR_LENGTH)
    plt.ylim(FLOOR_LENGTH, 0)
    plt.gca().set_aspect('equal', adjustable='box')

    # Plot obstacles (wall with opening)
    for i in range(FLOOR_LENGTH):
        for j in range(FLOOR_LENGTH):
            if obstacles[i][j] == HAS_OBSTACLE:
                plt.plot(i, j, 'ks', markersize=WALL_WIDTH)

    # Plot dirt locations
    for dirt in dirt_locations:
        plt.plot(dirt[0], dirt[1], 'ro', markersize=DUST_RADIUS)

    # Plot path if available
    if path:
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], 'b-', linewidth=LINE_WIDTH, label='Path')
        plt.legend()

    plt.show()


def tsp(dirt_locations, start):
    """Solve TSP for optimal dirt patch visiting order."""
    locations = [start] + dirt_locations
    n = len(locations)
    best_order = None
    best_cost = float('inf')

    for perm in permutations(range(1, n)):
        order = [0] + list(perm)
        cost = sum(math.dist(locations[order[i]], locations[order[i + 1]]) for i in range(n - 1))
        if cost < best_cost:
            best_cost = cost
            best_order = order

    return [locations[i] for i in best_order]


def run(dirt_locations: List[Tuple[int, int]], start: tuple, obstacles: np.array):
    # Initialize floor and dirt locations
    to_avoid = create_obstacle_matrix()
    optimal_order = tsp(dirt_locations, start)

    # Initialize RRT* and plan paths between consecutive points in optimal order
    total_path = []
    current_position = start

    for next_position in optimal_order[1:]:
        rrt_star = RRTStar(
            start=current_position, goal=next_position,
            width=FLOOR_LENGTH, height=FLOOR_LENGTH,
            obstacles=to_avoid
        )
        path = rrt_star.plan_to_point(next_position)
        if path:
            total_path.extend(path[1:])  # Avoid duplicate starting points
            current_position = next_position

    # Plot the results

    print(f'Final path: {total_path}')
    # plot(dirt_locations, obstacles, path=total_path)  # Plot with the final path
    return total_path
