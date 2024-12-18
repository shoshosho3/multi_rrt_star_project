import random
from consts import *
import math
import unit_transformations as ut
import numpy as np


def get_walls_and_rechargers(robot, battery_mode):
    """
    Get all walls in the simulation.
    :param robot: supervisor
    :param battery_mode: True if we are using the battery mode, False otherwise
    :return: wall objects
    """
    wall_number = 1
    recharger_number = 1
    walls = []
    rechargers = []

    # get all walls
    while robot.getFromDef(f'wall_{wall_number}') is not None:
        walls.append(robot.getFromDef(f'wall_{wall_number}'))
        wall_number += 1

    # handle rechargers
    while robot.getFromDef(f'recharger_{recharger_number}') is not None:
        if battery_mode:
            # add rechargers if we are using the battery mode
            rechargers.append(robot.getFromDef(f'recharger_{recharger_number}'))
            recharger_number += 1
        else:
            # remove rechargers if we are not using the battery mode
            robot.getFromDef(f'recharger_{recharger_number}').remove()
            recharger_number += 1

    return walls, rechargers


def get_wall_random_position_and_size(is_recharger=False):
    """
    Generate a random position and size for a wall.
    :param is_recharger: True if the wall is a recharger, False otherwise
    :return: random position and size for a wall
    """

    # generate random coordinates and sizes
    x = random.random() * GPS_LENGTH - FLOOR_ADD
    y = random.random() * GPS_LENGTH - FLOOR_ADD
    x_size = random.random() * MAX_WALL_SIZE + MIN_WALL_SIZE
    y_size = random.random() * MAX_WALL_SIZE + MIN_WALL_SIZE

    # rechargers have a fixed size
    if is_recharger:
        x_size = CHARGER_SIZE
        y_size = CHARGER_SIZE

    return x, y, x_size, y_size


def rect_distance(wall1, wall2):
    """
    Calculate the distance between two walls.
    :param wall1: wall 1
    :param wall2: wall 2
    :return: distance between the two walls
    """

    # unpack the walls into coordinates of the corners
    x1, y1, x1b, y1b = wall1
    x2, y2, x2b, y2b = wall2

    # Determine relative positions of the two rectangles
    left = x2b < x1
    right = x1b < x2
    bottom = y2b < y1
    top = y1b < y2

    # Calculate the distance based on the relative positions
    if top and left:
        return math.dist((x1, y1b), (x2b, y2))
    elif left and bottom:
        return math.dist((x1, y1), (x2b, y2b))
    elif bottom and right:
        return math.dist((x1b, y1), (x2, y2b))
    elif right and top:
        return math.dist((x1b, y1b), (x2, y2))
    elif left:
        return x1 - x2b
    elif right:
        return x2 - x1b
    elif bottom:
        return y1 - y2b
    elif top:
        return y2 - y1b
    else:
        # Rectangles intersect
        return 0


def min_dist_to_wall(wall1, prev_walls):
    """
    Calculate the minimum distance to an existing wall.
    :param wall1: new wall
    :param prev_walls: existing walls
    :return: minimum distance to an existing wall
    """
    min_dist = float('inf')
    for wall in prev_walls:
        dist = rect_distance(wall1, wall)
        if dist < min_dist:
            min_dist = dist
    return min_dist


def get_wall_corners(x, y, x_size, y_size):
    """
    Get the corners of a wall.
    :param x: middle x coordinate
    :param y: middle y coordinate
    :param x_size: x size
    :param y_size: y size
    :return: bottom left and top right corners of the wall
    """
    return x - x_size / 2, y - y_size / 2, x + x_size / 2, y + y_size / 2


def wall_gps_to_floor(walls):
    """
    Convert all wall GPS coordinates to floor coordinates.
    :param walls: list of walls in GPS coordinates
    :return: the walls in floor coordinates
    """
    wall_positions = []
    for wall in walls:
        x1, y1 = ut.gps_to_floor(wall[0:2])
        x2, y2 = ut.gps_to_floor(wall[2:4])
        wall_positions.append((x1, y1, x2, y2))
    return wall_positions


class Wall:
    """
    Class for handling walls in the simulation.
    """
    def __init__(self, robot, battery_mode):
        """
        Initialize the wall object.
        :param robot: supervisor
        :param battery_mode: True if we are using the battery mode, False otherwise
        """

        self.robot = robot

        # get all walls and rechargers if we are not using an empty world
        if robot is not None:
            self.wall_objects, self.rechargers = get_walls_and_rechargers(robot, battery_mode)
            self.all = self.wall_objects + self.rechargers
            self.positions = self.get_positions()

    def get_recharger_positions(self):
        """
        Get the positions of all rechargers in floor coordinates.
        :return: the positions of all rechargers in floor coordinates
        """
        locs = []
        for recharger in self.rechargers:
            locs.append(recharger.getField(TRANSLATION).getSFVec3f()[0:2])
        return locs

    def get_positions(self):
        """
        Get the positions of all walls in floor coordinates.
        :return: positions of all walls in floor coordinates
        """
        prev_walls = []
        for wall in self.all:
            x, y, _ = wall.getField(TRANSLATION).getSFVec3f()
            x_size, y_size, _ = wall.getField(SIZE).getSFVec3f()
            prev_walls.append(get_wall_corners(x, y, x_size, y_size))
        return wall_gps_to_floor(prev_walls)

    def set_random_positions(self):
        """
        Set positions for all walls randomly
        """
        prev_walls = []

        # set random positions for all walls
        for wall in self.wall_objects:
            x, y, x_size, y_size = get_wall_random_position_and_size()
            while min_dist_to_wall(get_wall_corners(x, y, x_size, y_size), prev_walls) < MIN_WALL_DISTANCE:
                x, y, x_size, y_size = get_wall_random_position_and_size()
            prev_walls.append(get_wall_corners(x, y, x_size, y_size))
            wall.getField(TRANSLATION).setSFVec3f([x, y, Z])
            wall.getField(SIZE).setSFVec3f([x_size, y_size, WALL_Z])
            wall.getField(ROTATION).setSFRotation([0, 0, 1, 0])

        # set random positions for all rechargers
        for recharger in self.rechargers:
            x, y, x_size, y_size = get_wall_random_position_and_size()
            while min_dist_to_wall(get_wall_corners(x, y, x_size, y_size), prev_walls) < MIN_WALL_DISTANCE:
                x, y, x_size, y_size = get_wall_random_position_and_size(True)
            prev_walls.append(get_wall_corners(x, y, CHARGER_SIZE, CHARGER_SIZE))
            recharger.getField(TRANSLATION).setSFVec3f([x, y, Z])
            recharger.getField(SIZE).setSFVec3f([CHARGER_SIZE, CHARGER_SIZE, CHARGER_SIZE])
            recharger.getField(ROTATION).setSFRotation([0, 0, 1, 0])

        self.positions = wall_gps_to_floor(prev_walls)

    def get_obstacle_matrix(self, delta):
        """
        Create a wall obstacle in the simulation environment
        :param delta: the distance to a physical wall we want to avoid
        """

        # initialize the matrix with zeros
        obstacles_matrix = np.zeros((FLOOR_LENGTH, FLOOR_LENGTH))

        # avoid colliding with side walls
        for i in range(delta):
            for j in range(FLOOR_LENGTH):
                obstacles_matrix[i][j] = HAS_OBSTACLE
                obstacles_matrix[j][i] = HAS_OBSTACLE

        for i in range(FLOOR_LENGTH - delta, FLOOR_LENGTH):
            for j in range(FLOOR_LENGTH):
                obstacles_matrix[i][j] = HAS_OBSTACLE
                obstacles_matrix[j][i] = HAS_OBSTACLE

        # if there are no walls in the simulation, return the matrix
        if self.robot is None:
            return obstacles_matrix

        # add the walls to the matrix
        for wall in self.positions:
            for x in range(wall[0] - delta, wall[2] + delta):
                for y in range(wall[3] - delta, wall[1] + delta):
                    if 0 <= x < FLOOR_LENGTH and 0 <= y < FLOOR_LENGTH:
                        obstacles_matrix[x][y] = HAS_OBSTACLE

        # return the matrix
        return obstacles_matrix
