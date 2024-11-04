from controller import Supervisor
import random
import numpy as np
from consts import *
import our_alg
import plots
import matplotlib.pyplot as plt


# Functions for dust generation and display updates
def create_dust(display, obstacles):
    """Create dust patches on the display, avoiding obstacles."""
    patches = []
    background = display.imageLoad(DUST_IMAGE)
    for _ in range(PATCH_NUMBER):
        x, y = random_position(DIRT_MIN_LOC, DIRT_MAX_LOC, obstacles, HAS_OBSTACLE)
        patches.append((x, y))
        display.imagePaste(background, x - 10, y - 10, False)
    return patches


def random_position(min_loc, max_loc, obstacles, obstacle_value):
    """Generate a random (x, y) position that avoids obstacles."""
    x, y = random.randint(min_loc, max_loc), random.randint(min_loc, max_loc)
    while obstacles[x][y] == obstacle_value:
        x, y = random.randint(min_loc, max_loc), random.randint(min_loc, max_loc)
    return x, y


def display_robot_positions(display, translations):
    """Display robots' positions on the floor."""
    for translation in translations:
        x = int(FLOOR_LENGTH * (translation[0] + GROUND_X / 2) / GROUND_X)
        y = int(FLOOR_LENGTH * (-translation[1] + GROUND_Y / 2) / GROUND_Y)
        display.fillOval(x, y, 2 * Radius, 2 * Radius)


# Conversion utilities between GPS and floor coordinates
def floor_to_gps(floor):
    """Convert floor coordinates to GPS coordinates."""
    return (
        floor[0] / FLOOR_LENGTH * GPS_LENGTH - FLOOR_ADD,
        FLOOR_ADD - (floor[1] / FLOOR_LENGTH * GPS_LENGTH)
    )


def gps_to_floor(gps):
    """Convert GPS coordinates to floor coordinates."""
    return (
        int((gps[0] + FLOOR_ADD) / GPS_LENGTH * FLOOR_LENGTH),
        int((FLOOR_ADD - gps[1]) / GPS_LENGTH * FLOOR_LENGTH)
    )


def distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


# Functions for robot initialization and positioning
def robot_position(robot, obstacles, prev_robots):
    """Place a robot randomly, avoiding obstacles and nearby robots."""
    x, y = random_position(ROBOT_START_MIN_LOC, ROBOT_START_MAX_LOC, obstacles, HAS_OBSTACLE)
    while any(distance((x, y), prev_robot) < ROBOT_START_MIN_DISTANCE for prev_robot in prev_robots):
        x, y = random_position(ROBOT_START_MIN_LOC, ROBOT_START_MAX_LOC, obstacles, HAS_OBSTACLE)
    x, y = floor_to_gps((x, y))
    robot.getField(TRANSLATION).setSFVec3f([x, y, Z])
    return x, y


def set_robot_positions(my_bots, obstacles):
    """Set positions for all robots, avoiding obstacles and each other."""
    prev_bots = []
    for bot in my_bots:
        prev_bots.append(robot_position(bot, obstacles, prev_bots))


# Wall and obstacle creation
def create_wall(walls, delta):
    """Create a wall obstacle in the simulation environment."""
    obstacles_matrix = np.zeros((FLOOR_LENGTH, FLOOR_LENGTH))
    for wall in walls:
        for x in range(wall[0] - delta, wall[2] + delta):
            for y in range(wall[3] - delta, wall[1] + delta):
                if 0 <= x < FLOOR_LENGTH and 0 <= y < FLOOR_LENGTH:
                    obstacles_matrix[x][y] = HAS_OBSTACLE
    return obstacles_matrix


# Robot data and path generation
def get_robots(robot):
    """Retrieve a list of all robot instances by their defined names."""
    return [robot.getFromDef(name) for name in ROBOT_NAMES]


def get_walls(robot):
    """Retrieve a list of all wall instances by their defined names."""
    return [robot.getFromDef(name) for name in WALL_NAMES]

def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def rect_distance(wall1, wall2):
    x1, y1, x1b, y1b = wall1
    x2, y2, x2b, y2b = wall2
    left = x2b < x1
    right = x1b < x2
    bottom = y2b < y1
    top = y1b < y2
    if top and left:
        return dist((x1, y1b), (x2b, y2))
    elif left and bottom:
        return dist((x1, y1), (x2b, y2b))
    elif bottom and right:
        return dist((x1b, y1), (x2, y2b))
    elif right and top:
        return dist((x1b, y1b), (x2, y2))
    elif left:
        return x1 - x2b
    elif right:
        return x2 - x1b
    elif bottom:
        return y1 - y2b
    elif top:
        return y2 - y1b
    else:             # rectangles intersect
        return 0


def dist_walls(wall1, prev_walls):
    """Calculate distance between a point and all walls."""
    min_dist = float('inf')
    for wall in prev_walls:
        dist = rect_distance(wall1, wall)
        if dist < min_dist:
            min_dist = dist
    return min_dist


def set_walls_positions(walls):
    """Set positions for all walls."""
    prev_walls = []
    for wall in walls:
        x = random.random() * 10 - 5
        y = random.random() * 10 - 5
        x_size = random.random() * 4.3 + 0.1
        y_size = random.random() * 4.3 + 0.1
        while dist_walls((x - x_size / 2, y - y_size / 2, x + x_size / 2, y + y_size / 2), prev_walls) < 2:
            x = random.random() * 10 - 5
            y = random.random() * 10 - 5
            x_size = random.random() * 4.3 + 0.1
            y_size = random.random() * 4.3 + 0.1
        prev_walls.append((x - x_size / 2, y - y_size / 2, x + x_size / 2, y + y_size / 2))
        wall.getField(TRANSLATION).setSFVec3f([x, y, Z])
        wall.getField('size').setSFVec3f([x_size, y_size, 2.4])
    print(f'prev_walls: {prev_walls}')
    wall_positions = []
    for wall in prev_walls:
        x1, y1 = gps_to_floor(wall[0:2])
        x2, y2 = gps_to_floor(wall[2:4])
        wall_positions.append((x1, y1, x2, y2))
    return wall_positions


def get_translations(my_bots):
    """Retrieve translation fields for each robot."""
    return [bot.getField(TRANSLATION) for bot in my_bots]


def get_paths(translations, patch_centers, walls):
    """Generate paths for robots from their positions to dust patches."""
    starts = [gps_to_floor(tuple(translation.getSFVec3f()[0:2])) for translation in translations]
    return our_alg.run(patch_centers, starts, create_wall, walls)


def send_message(msg, emitter):
    """Send a message as bytes to the emitter."""
    emitter.send(str(msg).encode('utf-8'))


# Main loop for simulation
def sim_loop(robot, display, translation_fields):
    """Main simulation loop to update robot positions and display."""
    while robot.step(TIME_STEP) != TERMINATE_TIME_STEP:
        display_robot_positions(display, [tf.getSFVec3f() for tf in translation_fields])


# Main function to initialize and start the simulation
def main():
    print('Starting supervisor...')

    # Setup simulation environment
    robot = Supervisor()

    display = robot.getDevice(DISPLAY)
    emitter = robot.getDevice(EMITTER)
    my_bots = get_robots(robot)
    walls = get_walls(robot)
    # # Initialize robot positions and dust patches
    walls = set_walls_positions(walls)
    print(walls)

    obstacles = create_wall(walls, DIRT_DELTA)
    set_robot_positions(my_bots, obstacles)
    translation_fields = get_translations(my_bots)
    dirt_locs = create_dust(display, obstacles)
    paths = get_paths(translation_fields, dirt_locs, walls)
    send_message(paths, emitter)

    # Display setup and simulation loop
    display.setAlpha(TRANSPARENCY)
    sim_loop(robot, display, translation_fields)
    robot.cleanup()


# Run the main function
if __name__ == "__main__":
    main()
