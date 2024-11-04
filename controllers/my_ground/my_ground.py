from controller import Supervisor
from consts import *
import our_alg
from wall_utils import Wall
from robot_utils import MyRobot
from random_positions import random_position


# Functions for dust generation and display updates
def create_dust(display, obstacles):
    """Create dust patches on the display, avoiding obstacles."""
    patches = []
    background = display.imageLoad(DUST_IMAGE)
    for _ in range(PATCH_NUMBER):
        x, y = random_position(DIRT_MIN_LOC, DIRT_MAX_LOC, obstacles, HAS_OBSTACLE)
        patches.append((x, y))
        display.imagePaste(background, x - DIRT_LOC_DELTA, y - DIRT_LOC_DELTA, False)
    return patches


def get_paths(robots, patch_centers, walls):
    """Generate paths for robots from their positions to dust patches."""
    starts = robots.get_positions()
    return our_alg.run(patch_centers, starts, walls)


def send_message(msg, emitter):
    """Send a message as bytes to the emitter."""
    emitter.send(str(msg).encode('utf-8'))


# Main loop for simulation
def sim_loop(robot, display, robots):
    """Main simulation loop to update robot positions and display."""
    while robot.step(TIME_STEP) != TERMINATE_TIME_STEP:
        robots.display_robot_positions(display)


# Main function to initialize and start the simulation
def main():
    print('Starting supervisor...')

    # Setup simulation environment
    robot = Supervisor()

    display = robot.getDevice(DISPLAY)
    emitter = robot.getDevice(EMITTER)
    my_bots = MyRobot(robot)

    walls = Wall(robot)
    walls.set_random_positions()
    obstacles = walls.get_obstacle_matrix(DIRT_DELTA)

    my_bots.set_random_positions(obstacles)
    dirt_locs = create_dust(display, obstacles)
    paths = get_paths(my_bots, dirt_locs, walls)
    send_message(paths, emitter)

    # Display setup and simulation loop
    display.setAlpha(TRANSPARENCY)
    sim_loop(robot, display, my_bots)
    robot.cleanup()


# Run the main function
if __name__ == "__main__":
    main()
