import random
import numpy as np
from consts import *
from controller import Robot
import math
import ast
from reg_rrt_star import run_rrt_star


def decide_speed(angle_diff):
    """
    Decides the left and right wheel speeds based on the angle difference
    :param angle_diff: angle difference between the robot's current bearing and the target angle
    :return: the left and right wheel speeds needed to turn the robot towards the target
    """

    if abs(angle_diff) < ANGLE_THRESHOLD:  # If the robot is roughly facing the target
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
    elif angle_diff > 0:  # Turn left
        left_speed = -HALF_SPEED
        right_speed = HALF_SPEED
    else:  # Turn right
        left_speed = HALF_SPEED
        right_speed = -HALF_SPEED
    return left_speed, right_speed


class GotoRobot:
    def __init__(self):
        """
        Initializes the robot's devices and sensors.
        """

        # Initialize Webots robot instance
        self.robot = Robot()

        self.name = self.robot.getName()

        # Initialize GPS and Compass sensors
        self.gps = self.robot.getDevice(GPS)
        self.gps.enable(TIME_STEP)

        self.compass = self.robot.getDevice(COMPASS)
        self.compass.enable(TIME_STEP)

        # Initialize motors (left and right wheels)
        self.left_motor = self.robot.getDevice(LEFT_WHEEL_MOTOR)
        self.right_motor = self.robot.getDevice(RIGHT_WHEEL_MOTOR)

        # Set motors to velocity control mode
        self.left_motor.setPosition(INF)
        self.right_motor.setPosition(INF)
        self.left_motor.setVelocity(NULL_SPEED)
        self.right_motor.setVelocity(NULL_SPEED)

        # Initialize receiver device
        self.receiver = self.robot.getDevice(RECEIVER)
        self.receiver.enable(TIME_STEP)

        # Initialize position sensors
        self.left_position_sensor = self.robot.getDevice(LEFT_WHEEL_SENSOR)
        self.right_position_sensor = self.robot.getDevice(RIGHT_WHEEL_SENSOR)
        self.left_position_sensor.enable(TIME_STEP)
        self.right_position_sensor.enable(TIME_STEP)

        # Initialize battery parameters
        self.battery_level = FULL_BATTERY  # Start with full battery
        self.critical_battery_level = CRITICAL_BATTERY_LEVEL  # Critical threshold (15% of capacity)
        self.consumption_rate = random.uniform(CONSUMPTION_RATE_LOWER, CONSUMPTION_RATE_UPPER)
        self.nearby_charging_station = False  # Mocked initially

    def get_gps_position(self):
        """
        Returns the (x, y) coordinates from GPS
        :return: x, y coordinates
        """

        pos = self.gps.getValues()
        return pos[0], pos[1]  # We only need the x, y coordinates

    def step(self):
        """
        Steps the simulation by TIME_STEP
        """
        if self.robot.step(TIME_STEP) == -1:
            exit(0)

    def stop(self):
        """
        Stops the robot
        """
        self.left_motor.setVelocity(NULL_SPEED)
        self.right_motor.setVelocity(NULL_SPEED)

    def get_bearing(self):
        """
        Returns the bearing (angle) in radians from the compass.
        """
        compass_values = self.compass.getValues()
        # Calculate the angle from the compass readings
        angle = math.atan2(compass_values[0], compass_values[1])
        return angle

    def calculate_angle_to_target(self, target_position):
        """
        Calculates the angle from the current position to the target
        :param target_position: The target position
        :return: The angle in radians
        """
        current_position = self.get_gps_position()
        delta_x = target_position[X_POS] - current_position[X_POS]
        delta_y = target_position[Y_POS] - current_position[Y_POS]
        return math.atan2(delta_y, delta_x)

    def passive_wait(self, seconds):
        """
        Waits for the given number of seconds
        :param seconds: number of seconds to wait
        """
        start_time = self.robot.getTime()
        while self.robot.getTime() < start_time + seconds:
            self.step()

    def calculate_distance(self, target_position):
        """
        Calculates the distance between the current position and the target position
        :param target_position: the target position
        :return: the distance between current and target positions
        """
        current_position = self.get_gps_position()
        return math.dist(current_position[0:2], target_position[0:2])

    def update_battery(self):
        """
        Updates the battery level based on the consumption rate
        """

        consumption = random.uniform(CONSUMPTION_LOWER, CONSUMPTION_UPPER) * self.consumption_rate
        self.battery_level -= consumption

    def check_battery(self, recharge_locations, cost_left, target, obstacle_matrix):
        """
        Checks the battery level and goes to the charging station if necessary
        :param recharge_locations: the locations of the charging stations
        :param cost_left: cost left to reach all targets
        :param target: current target location
        :param obstacle_matrix: matrix of obstacles
        """

        # getting the minimum distance to the nearest charging station
        min_dist = self.min_distance(recharge_locations)[0]

        # if the battery is low and the robot closer to the charging station than all targets
        if (min_dist < cost_left + self.calculate_distance(target) and
                self.battery_level <= self.min_distance(recharge_locations)[0] * self.consumption_rate * 60):
            print(f"{self.name} warning: Low battery! Returning to charging station.")
            self.return_to_charge(recharge_locations, obstacle_matrix)

        # if the battery is moderately low and the robot is close to a charging station
        elif self.battery_level <= self.critical_battery_level and min_dist < CHARGING_DISTANCE:
            print(f"{self.name} battery is moderately low. Opportunity to recharge nearby.")
            self.charge_battery()

    def min_distance(self, recharge_locations):
        """
        Finds the minimum distance to the nearest charging station
        :param recharge_locations: locations of the charging stations
        :return: the minimum distance and the location of the nearest charging station
        """
        min_loc = recharge_locations[0]
        min_dist = self.calculate_distance(min_loc)
        for loc in recharge_locations:
            # dist = abs(loc[0] - self.get_gps_position()[0]) + abs(loc[1] - self.get_gps_position()[1])
            dist = self.calculate_distance(loc)
            if dist < min_dist:
                min_dist = dist
                min_loc = loc
        return min_dist, min_loc

    def return_to_charge(self, recharge_locations, obstacle_matrix):
        """
        Returns the robot to the charging station if possible
        :param recharge_locations: the locations of the charging stations
        :param obstacle_matrix: matrix of obstacles
        """
        print(f"{self.name} returning to charging station...")
        min_loc = self.min_distance(recharge_locations)[1]

        # getting the path to the charging station
        path = run_rrt_star(self.get_gps_position(), min_loc, obstacle_matrix)

        # checking if the path is found
        if path is None:
            print(f"{self.name} could not find a path to the charging station.")
            return

        # going to the charging station
        for loc in path:
            self.goto(loc, recharge_locations, obstacle_matrix, to_charge=True)

        # charging the battery
        if self.calculate_distance(min_loc) < CHARGING_DISTANCE:
            self.charge_battery()

    def charge_battery(self):
        print(f"{self.name} charging battery...")
        self.passive_wait(CHARGING_TIME / (FULL_BATTERY - self.battery_level))
        self.battery_level = FULL_BATTERY

    def goto(self, target_position, recharge_locations, obstacle_matrix, to_charge=False, cost_left=float('inf')):
        """
        Moves the robot towards the target GPS position.
        :param target_position: the target position
        :param recharge_locations: the locations of the charging stations
        :param obstacle_matrix: matrix of obstacles
        :param to_charge: whether the robot is going to charge
        :param cost_left: cost left to reach all targets
        """
        while self.robot.step(TIME_STEP) != TERMINATE_TIME_STEP:

            # Get the robot's current bearing and angle to the target
            current_bearing = self.get_bearing()
            target_angle = self.calculate_angle_to_target(target_position)

            # Calculate the angle difference
            angle_diff = target_angle - current_bearing

            # Normalize the angle difference to [-pi, pi]
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            # Adjust wheel speeds based on the angle difference
            left_speed, right_speed = decide_speed(angle_diff)

            # Set motor velocities
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)

            # Check if the robot is close to the target position
            distance = self.calculate_distance(target_position)
            if distance < DISTANCE_THRESHOLD:  # Stop if the robot is close enough to the target
                break

            # check if we are in battery mode
            if len(recharge_locations) > 0:

                # Update battery level
                if left_speed != 0 or right_speed != 0:
                    self.update_battery()

                # stop if there is no battery left
                if self.battery_level <= 0:
                    print(f'{self.name} had to stop due to battery')
                    break

                # check battery if we are not already going to charge it
                if not to_charge:
                    self.check_battery(recharge_locations, cost_left, target_position, obstacle_matrix)

        # Stop the robot
        self.stop()

    def receive_path(self, start):
        """
        Receives the paths from the supervisor
        :param start: start position of the robot
        :return: the path to the target position
        """
        while self.receiver.getQueueLength() > 0:

            # get the message from the supervisor
            message = self.receiver.getString()
            paths, recharge_locs, mat = ast.literal_eval(message)  # Convert string back to list
            print(f"{self.name} received paths")

            for path in paths:

                # check if the path is empty
                if len(path) == 0:
                    continue

                # get start of path and start of robot
                x1, y1 = path[0]
                x2, y2 = start

                # Check if the path is for the robot
                if abs(x1 - x2) <= 1 and abs(y1 - y2) <= 1:
                    return path, recharge_locs, mat
            self.receiver.nextPacket()  # Move to next packet in the queue

        # Return None if no path is found
        return None, None, None
