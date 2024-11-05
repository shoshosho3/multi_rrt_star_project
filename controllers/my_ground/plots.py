import matplotlib.pyplot as plt
import numpy as np
from consts import *


def plot_obstacles(obstacles):
    """
    Plot the obstacles on the floor.
    :param obstacles: 2D array of obstacles on the floor.
    """
    for i in range(FLOOR_LENGTH):
        for j in range(FLOOR_LENGTH):
            if obstacles[i][j] == HAS_OBSTACLE:
                plt.plot(i, j, 'ks', markersize=WALL_WIDTH)


def plot_dirt(dirt_locations):
    """
    Plot the dirt patches on the floor
    :dirt_locations: 2D array of dirt patches on the floor.
    """
    for i, dirt in enumerate(dirt_locations):
        if i == 0:
            plt.plot(dirt[0], dirt[1], 'ro', markersize=DUST_RADIUS, label='Dirt')
        else:
            plt.plot(dirt[0], dirt[1], 'ro', markersize=DUST_RADIUS)


def plot_path(i, start, path=None):
    """
    Plot the path of the robot
    :param i: Index of the robot
    :param start: Starting position of the robot
    :param path: Path of the robot
    """

    if i == 0:
        plt.plot(start[0], start[1], 'go', markersize=START_RADIUS, label='Start')
    else:
        plt.plot(start[0], start[1], 'go', markersize=START_RADIUS)

    # Plot path if available
    if path:
        path = np.array(path)
        if i == 0:
            plt.plot(path[:, 0], path[:, 1], 'b-', linewidth=LINE_WIDTH, label='Path')
        else:
            plt.plot(path[:, 0], path[:, 1], 'b-', linewidth=LINE_WIDTH)


def plot_paths(paths, starts):
    """
    Plot the paths of the robots
    :param paths: List of paths of the robots
    :param starts: List of starting positions of the robots
    """
    for i in range(len(paths)):
        # paths[i] = [starts[i]] + paths[i]
        print(f'Final path for {i + 1}th agent: {paths[i]}')
        plot_path(i, starts[i], path=paths[i])


def create_plot(obstacles, dirt_locations, starts, paths, solver, plot_solver=False):
    """
    Create a plot of the floor with the obstacles, dirt patches, and paths
    :param obstacles: 2D array of obstacles on the floor.
    :param dirt_locations: 2D array of dirt patches on the floor.
    :param starts: List of starting positions of the robots
    :param paths: List of paths of the robots
    :param solver: Solver object
    :param plot_solver: Boolean to plot the solver trees and connections
    """
    plt.figure(figsize=(FIG_SIZE, FIG_SIZE))
    plt.xlim(0, FLOOR_LENGTH)
    plt.ylim(FLOOR_LENGTH, 0)
    plt.gca().set_aspect('equal', adjustable='box')

    if plot_solver:
        solver.plot_all_trees()
        solver.plot_connections()

    # plotting the different elements
    plot_obstacles(obstacles)
    plot_dirt(dirt_locations)
    plot_paths(paths, starts)

    plt.legend()
    plt.show()
