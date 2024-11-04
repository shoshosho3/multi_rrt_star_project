import random


def random_position(min_loc, max_loc, obstacles, obstacle_value):
    """Generate a random (x, y) position that avoids obstacles."""
    x, y = random.randint(min_loc, max_loc), random.randint(min_loc, max_loc)
    while obstacles[x][y] == obstacle_value:
        x, y = random.randint(min_loc, max_loc), random.randint(min_loc, max_loc)
    return x, y
