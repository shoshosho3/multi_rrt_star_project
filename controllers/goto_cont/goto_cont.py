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
    path = robot.receive_path(ut.gps_to_floor(robot.get_gps_position()))
    additional_time = 0
    while path is None:
        robot.passive_wait(WAIT_TIME + additional_time)
        additional_time += 1
        path = robot.receive_path(ut.gps_to_floor(robot.get_gps_position()))
    return path


def goto_path(robot, path):
    """
    Goes to the path
    :param robot: robot object
    :param path: path of coordinates
    """
    for loc in path:
        next_pos = ut.floor_to_gps(loc)
        robot.goto(next_pos)


def main():
    # initialize the robot
    robot = GotoRobot()

    # wait for the path
    path = wait_for_path(robot)

    # go to the path
    goto_path(robot, path)


if __name__ == "__main__":
    main()
