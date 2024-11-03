import matplotlib.pyplot as plt
import numpy as np
from consts import *


def plot_obstacles(obstacles):
    for i in range(FLOOR_LENGTH):
        for j in range(FLOOR_LENGTH):
            if obstacles[i][j] == HAS_OBSTACLE:
                plt.plot(i, j, 'ks', markersize=WALL_WIDTH)


def plot_dirt(dirt_locations):
    for i, dirt in enumerate(dirt_locations):
        if i == 0:
            plt.plot(dirt[0], dirt[1], 'ro', markersize=DUST_RADIUS, label='Dirt')
        else:
            plt.plot(dirt[0], dirt[1], 'ro', markersize=DUST_RADIUS)


def plot_path(i, start, path=None):
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
    for i in range(len(paths)):
        paths[i] = [starts[i]] + paths[i]
        print(f'Final path for {i + 1}th agent: {paths[i]}')
        plot_path(i, starts[i], path=paths[i])


def create_plot(obstacles, dirt_locations, starts, paths):
    plt.figure(figsize=(FIG_SIZE, FIG_SIZE))
    plt.xlim(0, FLOOR_LENGTH)
    plt.ylim(FLOOR_LENGTH, 0)
    plt.gca().set_aspect('equal', adjustable='box')

    # plotting the different elements
    plot_obstacles(obstacles)
    plot_dirt(dirt_locations)
    plot_paths(paths, starts)

    plt.legend()
    plt.show()
