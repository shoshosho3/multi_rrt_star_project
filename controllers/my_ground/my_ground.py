from controller import Supervisor
from consts import *
import our_alg
from wall_utils import Wall
from robot_utils import MyRobot
from random_positions import random_position
import random


# Functions for dust generation and display updates
def create_dust(display, obstacles):
    """
    Create dust patches on the display, avoiding obstacles.
    :param display: Display device to paste dust patches on.
    :param obstacles: Matrix of obstacles to avoid.
    """
    patches = []
    background_dust = display.imageLoad(DUST_IMAGE)
    for _ in range(PATCH_NUMBER):
        x, y = random_position(DIRT_MIN_LOC, DIRT_MAX_LOC, obstacles, HAS_OBSTACLE)
        patches.append((x, y))
        display.imagePaste(background_dust, x - DIRT_LOC_DELTA, y - DIRT_LOC_DELTA, False)
    return patches


def get_paths(robots: MyRobot, patch_centers: list, walls: Wall):
    """
    Generate paths for robots from their positions to dust patches
    :param robots: List of robots to generate paths for.
    :param patch_centers: List of dust patch centers.
    :param walls: Wall object to avoid collisions with walls.
    """
    starts = robots.get_positions()
    return our_alg.run(patch_centers, starts, walls)


def send_message(msg, emitter):
    """
    Send a message as bytes to the emitter
    :param msg: Message to send.
    :param emitter: Emitter device to send the message.
    """
    emitter.send(str(msg).encode('utf-8'))


# Main loop for simulation
def sim_loop(robot: Supervisor, display, robots: MyRobot):
    """
    Main simulation loop to update robot positions and display
    :param robot: Supervisor object to control simulation.
    :param display: Display device to update.
    :param robots: MyRobot object to update positions.
    """
    while robot.step(TIME_STEP) != TERMINATE_TIME_STEP:
        robots.display_robot_positions(display)


# Main function to initialize and start the simulation
def main():

    random.seed(0)

    print('Starting supervisor...')

    battery_mode = True

    # Setup simulation environment
    robot = Supervisor()

    # Initialize devices and objects
    display = robot.getDevice(DISPLAY)
    emitter = robot.getDevice(EMITTER)
    my_bots = MyRobot(robot)

    # initialize random walls and create obstacle matrix based on them
    walls = Wall(robot, battery_mode)
    walls.set_random_positions()
    obstacles = walls.get_obstacle_matrix(DIRT_DELTA)

    # Set random positions for robots and create dust patches
    my_bots.set_random_positions(obstacles)
    dirt_locs = create_dust(display, obstacles)
    recharge_locs = walls.get_recharger_positions()
    paths = get_paths(my_bots, dirt_locs, walls)
    obstacles = [list(row) for row in obstacles]
    send_message((paths, recharge_locs, obstacles), emitter)

    # Display setup and simulation loop
    display.setAlpha(TRANSPARENCY)
    sim_loop(robot, display, my_bots)
    robot.cleanup()


# Run the main function
if __name__ == "__main__":
    main()
