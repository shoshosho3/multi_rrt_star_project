import math
from goto_robot import GotoRobot
from consts import *
import unit_transformations as ut


def wait_for_path(robot):
    """
    Waits for the path to be received from supervisor
    :param robot: robot object
    :return: the path
    """
    robot.passive_wait(WAIT_TIME)
    path, recharge, mat = robot.receive_path(ut.gps_to_floor(robot.get_gps_position()))
    additional_time = 0
    while path is None:
        robot.passive_wait(WAIT_TIME + additional_time)
        additional_time += 1
        path, recharge, mat = robot.receive_path(ut.gps_to_floor(robot.get_gps_position()))
    return path, recharge, mat


def get_all_costs_left(path):
    """
    This function calculates the cost left for each point in the path
    :param path: path of coordinates
    :return: the costs left
    """
    costs_left = []
    for i in range(1, len(path) + 1):
        last = 0 if i == 1 else costs_left[i - 2]
        costs_left.append(math.dist(path[len(path) - i], path[len(path) - i - 1]) + last)
    costs_left.reverse()
    return costs_left


def goto_path(robot, path, recharge, obstacle_matrix):
    """
    Goes to the path
    :param robot: robot object
    :param path: path of coordinates
    :param recharge: coordinates of recharges
    :param obstacle_matrix: matrix of the obstacles
    """

    # getting recharges locations
    recharges = []
    for loc in recharge:
        recharges.append(loc)

    # getting cost of path left for each point
    costs = get_all_costs_left(path)

    # going to each point in the path
    for i, loc in enumerate(path):
        next_pos = ut.floor_to_gps(loc)
        if i == len(path) - 1:
            robot.goto(next_pos, recharges, obstacle_matrix, cost_left=0)
        else:
            robot.goto(next_pos, recharges, obstacle_matrix, cost_left=costs[i + 1])


def main():
    # initialize the robot
    robot = GotoRobot()

    # wait for the path
    path, recharge, mat = wait_for_path(robot)

    # go to the path
    goto_path(robot, path, recharge, mat)


if __name__ == "__main__":
    main()
