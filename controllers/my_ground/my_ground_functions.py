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

    # start time of the simulation
    start = robot.getTime()

    # Main loop for simulation
    while not robots.display_robot_positions(display) and robot.getTime() - start < TIMEOUT:
        robot.step(TIME_STEP)

    # check if the simulation timed out
    if robot.getTime() - start >= TIMEOUT:
        print("Simulation timed out.")

    # let the robots move for a few more steps to ensure they reach their goal
    for i in range(END_STEP_NUM):
        robot.step(TIME_STEP)

    # print the time it took for the simulation to finish
    print(f"Simulation finished in {round(time.time() - start, 2)} seconds.")


def log_to_csv(filename, patch_number, robot_number, seed, alg_time, success, run_time, total_time, price):
    """
    Append data to a CSV file
    :param filename: file name
    :param patch_number: patch number
    :param robot_number: robot number
    :param seed: random seed
    :param alg_time: algorithm time
    :param success: did the algorithm succeed
    :param run_time: run time
    :param total_time: total time
    :param price: cost of the solution
    """

    # Define the data to be appended
    data = [patch_number, robot_number, seed, alg_time, success, run_time, total_time, price]

    # Open the file in append mode
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        # Write the data row
        writer.writerow(data)
    print(f"Data appended to {filename}: {data}")


def remove_robot(robot, robot_number):
    """
    Remove a robot from the simulation if it exists
    :param robot: robot supervisor
    :param robot_number: robot number iteration
    """
    try:
        robot.getFromDef(f'robot_{MAX_ROBOTS - robot_number + 1}').remove()
    except:
        pass


def run_simulations(robot, display, emitter, battery_mode, min_robots=MIN_ROBOTS, max_robots=MAX_ROBOTS,
                    patch_numbers=PATCH_NUMBERS, seeds=SEEDS, save_mode=False):

    """
    Run the simulations for the given parameters
    :param robot: supervisor
    :param display: display
    :param emitter: emitter device
    :param battery_mode: true if the battery mode is on, false otherwise
    :param min_robots: minimum number of robots
    :param max_robots: maximum number of robots
    :param patch_numbers: patch numbers
    :param seeds: seeds
    :param save_mode: true if the data should be saved to a CSV file
    :return: list of robots
    """

    # Check if the number of robots is within the limits
    min_robots = max(min_robots, MIN_ROBOTS)
    max_robots = min(max_robots, MAX_ROBOTS)

    # Loop through the number of robots
    for robot_number in range(MAX_ROBOTS - min_robots + 1):

        # Remove the robot if it exists
        remove_robot(robot, robot_number)

        # Check if the robot number is within the wanted range
        if robot_number < MAX_ROBOTS - max_robots:
            continue

        print(f'----------------------------------- Robot Number: {MAX_ROBOTS - robot_number} '
              f'-----------------------------------')

        # Loop through the patch numbers
        for patch_number in patch_numbers:

            print(
                f'----------------------------------- Patch Number: {patch_number} -----------------------------------')

            # Loop through the seeds
            for seed in seeds:

                print(f'----------------------------------- Seed: {seed} -----------------------------------')

                random.seed(seed)
                np.random.seed(seed)

                # Initialize the simulation
                start_time = time.time()
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

                # Run the rrt* algorithm to get the paths
                alg_start_time = time.time()
                did_succeed, paths, price = get_paths(my_bots, dirt_locs, walls)
                alg_time = time.time() - alg_start_time

                # set the obstacles for message to robots in battery mode
                new_obs = walls.get_obstacle_matrix(NEW_OBS_DELTA)
                new_obs = [list(row) for row in new_obs]
                new_obs = [row[::-1] for row in new_obs]

                # Skip the simulation if the battery mode is on and the algorithm did not succeed
                # or the price is too high
                if battery_mode and (not did_succeed or price >= PRICE_LIMIT):
                    continue

                # send the paths to the robots for execution
                send_message((paths, recharge_locs, new_obs, seed), emitter)

                # Display setup and simulation loop
                display.setAlpha(TRANSPARENCY)
                run_start_time = robot.getTime()
                sim_loop(robot, display, my_bots)
                run_time = robot.getTime() - run_start_time
                total_time = time.time() - start_time

                # Log the data to a CSV file
                if save_mode:
                    if battery_mode:
                        log_to_csv(CSV_BATTERY_NAME, patch_number, robot_number, seed,
                                   alg_time, did_succeed, run_time, total_time, price)
                    else:
                        log_to_csv(CSV_NAME, patch_number, robot_number, seed,
                                   alg_time, did_succeed, run_time, total_time, price)

                # Reset the physics of the robots
                for bot in my_bots.robots:
                    bot.resetPhysics()

    return my_bots


def terminal_message(emitter, my_bots):
    """
    Send a message to the robots to terminate all simulations
    :param emitter: emitter device
    :param my_bots: list of robots
    """
    send_message(([[TERMINATE_TIME_STEP] for _ in range(len(my_bots.robots))], None, None, 0), emitter)
