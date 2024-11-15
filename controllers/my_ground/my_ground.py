import time
import numpy as np
from controller import Supervisor
from consts import *
import our_alg
from wall_utils import Wall
from robot_utils import MyRobot
from random_positions import random_position
import random
import csv
import consts


# Functions for dust generation and display updates
def create_dust(display, obstacles, patch_number):
    """
    Create dust patches on the display, avoiding obstacles.
    :param display: Display device to paste dust patches on.
    :param obstacles: Matrix of obstacles to avoid.
    """
    patches = []
    background_dust = display.imageLoad(DUST_IMAGE)
    for _ in range(patch_number):
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
    # while robot.step(TIME_STEP) != TERMINATE_TIME_STEP:
    #     robots.display_robot_positions(display)

    start = robot.getTime()

    while not robots.display_robot_positions(display) and robot.getTime() - start < 600:
        robot.step(TIME_STEP)

    if robot.getTime() - start >= 600:
        print("Simulation timed out.")

    for i in range(2):
        robot.step(TIME_STEP)

    print(f"Simulation finished in {round(time.time() - start, 2)} seconds.")


def log_to_csv(filename, patch_number, robot_number, seed, alg_time, success, run_time, total_time, price):
    # Define the data to be appended
    data = [patch_number, robot_number, seed, alg_time, success, run_time, total_time, price]

    # Open the file in append mode
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        # Write the data row
        writer.writerow(data)
    print(f"Data appended to {filename}: {data}")


# Main function to initialize and start the simulation
def main():
    print('Starting supervisor...')

    # Setup simulation environment
    robot = Supervisor()

    # Initialize devices and objects
    display = robot.getDevice(DISPLAY)
    emitter = robot.getDevice(EMITTER)

    for robot_number in range(2):

        try:
            robot.getFromDef(f'robot_{6 - robot_number}').remove()
        except:
            pass

        # if robot_number < 2:
        #     continue

        print(f'----------------------------------- Robot Number: {5 - robot_number} '
              f'-----------------------------------')

        for patch_number in range(1, 16):

            print(
                f'----------------------------------- Patch Number: {patch_number} -----------------------------------')

            for seed in range(20):
                print(f'----------------------------------- Seed: {seed} -----------------------------------')
                random.seed(seed)
                np.random.seed(seed)

                start_time = time.time()

                battery_mode = False

                my_bots = MyRobot(robot)

                # initialize random walls and create obstacle matrix based on them
                walls = Wall(robot, battery_mode)
                walls.set_random_positions()
                obstacles = walls.get_obstacle_matrix(DIRT_DELTA)

                # Set random positions for robots and create dust patches
                my_bots.set_random_positions(obstacles)
                dirt_locs = create_dust(display, obstacles, patch_number)
                my_bots.set_dirt_patches(dirt_locs)
                recharge_locs = walls.get_recharger_positions()
                alg_start_time = time.time()
                did_succeed, paths, price = get_paths(my_bots, dirt_locs, walls)
                alg_time = time.time() - alg_start_time
                obstacles = [list(row) for row in obstacles]

                send_message((paths, recharge_locs, obstacles), emitter)

                # Display setup and simulation loop
                display.setAlpha(TRANSPARENCY)
                # run_start_time = time.time()
                run_start_time = robot.getTime()
                sim_loop(robot, display, my_bots)
                # run_time = time.time() - run_start_time
                run_time = robot.getTime() - run_start_time
                total_time = time.time() - start_time
                log_to_csv(CSV_NAME, patch_number, robot_number, seed,
                           alg_time, did_succeed, run_time, total_time, price)

                for bot in my_bots.robots:
                    bot.resetPhysics()

    send_message(([[TERMINATE_TIME_STEP] for _ in range(len(my_bots.robots))], None, None), emitter)


# Run the main function
if __name__ == "__main__":
    main()
