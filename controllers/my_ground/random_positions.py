import random


def random_position(min_loc, max_loc, obstacles, obstacle_value):
    """
    Generate a random (x, y) position that avoids obstacles.
    :param min_loc: minimum location value
    :param max_loc: maximum location value
    :param obstacles: 2D list of obstacles
    :param obstacle_value: value of obstacle
    :return: (x, y) position
    """
    x, y = random.randint(min_loc, max_loc), random.randint(min_loc, max_loc)
    while obstacles[x][y] == obstacle_value:
        x, y = random.randint(min_loc, max_loc), random.randint(min_loc, max_loc)
    return x, y
