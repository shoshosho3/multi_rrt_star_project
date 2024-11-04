from controller import Supervisor
from consts import *
import random
import math
import unit_transformations as ut
from random_positions import random_position


def get_robots(robot):
    i = 1
    walls = []
    while robot.getFromDef(f'robot_{i}') is not None:
        walls.append(robot.getFromDef(f'robot_{i}'))
        i += 1
    return walls


def robot_position(robot, obstacles, prev_robots):
    """Place a robot randomly, avoiding obstacles and nearby robots."""
    x, y = random_position(ROBOT_START_MIN_LOC, ROBOT_START_MAX_LOC, obstacles, HAS_OBSTACLE)
    while any(math.dist((x, y), prev_robot) < ROBOT_START_MIN_DISTANCE for prev_robot in prev_robots):
        x, y = random_position(ROBOT_START_MIN_LOC, ROBOT_START_MAX_LOC, obstacles, HAS_OBSTACLE)
    x, y = ut.floor_to_gps((x, y))
    robot.getField(TRANSLATION).setSFVec3f([x, y, Z])
    return x, y


class MyRobot:

    def __init__(self, robot: Supervisor):
        self.robots = get_robots(robot)

    def get_positions(self, is_gps=False):
        if is_gps:
            return [robot.getField(TRANSLATION).getSFVec3f()[0:2] for robot in self.robots]
        return [ut.gps_to_floor(robot.getField(TRANSLATION).getSFVec3f()[0:2]) for robot in self.robots]

    def set_random_positions(self, obstacles):
        """Set positions for all robots, avoiding obstacles and each other."""
        prev_bots = []
        for bot in self.robots:
            prev_bots.append(robot_position(bot, obstacles, prev_bots))

    def display_robot_positions(self, display):
        """Display robots' positions on the floor."""
        translations = self.get_positions(True)
        for translation in translations:
            x = int(FLOOR_LENGTH * (translation[0] + GROUND_X / 2) / GROUND_X)
            y = int(FLOOR_LENGTH * (-translation[1] + GROUND_Y / 2) / GROUND_Y)
            display.fillOval(x, y, 2 * Radius, 2 * Radius)
