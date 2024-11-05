import numpy as np
from controller import Robot
import math
import ast
import gpt_rrt_star
from consts import *


class GotoRobot:
    def __init__(self):
        # Initialize Webots robot instance
        self.robot = Robot()

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

        self.receiver = self.robot.getDevice(RECEIVER)
        self.receiver.enable(TIME_STEP)

        self.left_position_sensor = self.robot.getDevice(LEFT_WHEEL_SENSOR)
        self.right_position_sensor = self.robot.getDevice(RIGHT_WHEEL_SENSOR)
        self.left_position_sensor.enable(TIME_STEP)
        self.right_position_sensor.enable(TIME_STEP)

    def get_gps_position(self):
        """Returns the (x, y) coordinates from GPS."""
        pos = self.gps.getValues()
        return pos[0], pos[1]  # We only need the x, y coordinates

    def step(self):
        if self.robot.step(TIME_STEP) == -1:
            exit(0)

    def stop(self):
        self.left_motor.setVelocity(NULL_SPEED)
        self.right_motor.setVelocity(NULL_SPEED)

    def get_bearing(self):
        """Returns the bearing (angle) in radians from the compass."""
        compass_values = self.compass.getValues()
        # Calculate the angle from the compass readings
        angle = math.atan2(compass_values[0], compass_values[1])
        return angle

    def calculate_angle_to_target(self, target_position):
        """Calculates the angle from the current position to the target."""
        current_position = self.get_gps_position()
        delta_x = target_position[X_POS] - current_position[X_POS]
        delta_y = target_position[Y_POS] - current_position[Y_POS]
        return math.atan2(delta_y, delta_x)

    def passive_wait(self, seconds):
        start_time = self.robot.getTime()
        while self.robot.getTime() < start_time + seconds:
            self.step()

    def decide_speed(self, angle_diff):
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

    def calculate_distance(self, target_position):
        current_position = self.get_gps_position()
        return math.sqrt((target_position[X_POS] - current_position[X_POS]) ** 2 +
                         (target_position[Y_POS] - current_position[Y_POS]) ** 2)

    def goto(self, target_position):
        """Moves the robot towards the target GPS position."""
        while self.robot.step(TIME_STEP) != TERMINATE_TIME_STEP:

            # Get the robot's current bearing and angle to the target
            current_bearing = self.get_bearing()
            target_angle = self.calculate_angle_to_target(target_position)

            # Calculate the angle difference
            angle_diff = target_angle - current_bearing

            # Normalize the angle difference to [-pi, pi]
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            # Adjust wheel speeds based on the angle difference
            left_speed, right_speed = self.decide_speed(angle_diff)

            # Set motor velocities
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)

            # Check if the robot is close to the target position
            distance = self.calculate_distance(target_position)
            if distance < DISTANCE_THRESHOLD:  # Stop if the robot is close enough to the target
                break

        # Stop the robot
        self.stop()

    def receive_dust_locations(self):
        dust_locations = []
        while self.receiver.getQueueLength() > 0:
            # Receive the dust locations from the supervisor
            message = self.receiver.getString()
            dust_locations = ast.literal_eval(message)  # Convert string back to list
            print(f"Received dust locations: {dust_locations}")
            self.receiver.nextPacket()  # Move to next packet in the queue
        return dust_locations

    def receive_path(self, start):
        while self.receiver.getQueueLength() > 0:
            message = self.receiver.getString()
            paths = ast.literal_eval(message)  # Convert string back to list
            name = self.robot.getName()
            print(f"{name} received paths")
            for path in paths:
                if len(path) == 0:
                    continue
                x1, y1 = path[0]
                x2, y2 = start
                if abs(x1 - x2) <= 1 and abs(y1 - y2) <= 1:
                    return path
            self.receiver.nextPacket()  # Move to next packet in the queue
        return None


def create_wall():
    obstacles_matrix = np.zeros((FLOOR_LENGTH, FLOOR_LENGTH))
    for x_position in range(WALL_X_LEFT, WALL_X_RIGHT):
        for y_position in list(range(0, WALL_Y_UP)) + list(range(WALL_Y_DOWN, FLOOR_LENGTH)):
            obstacles_matrix[x_position][y_position] = HAS_OBSTACLE
    return obstacles_matrix


def gps_to_floor(gps):
    return int((gps[0] + FLOOR_ADD) / GPS_LENGTH * FLOOR_LENGTH), int((FLOOR_ADD - gps[1]) / GPS_LENGTH * FLOOR_LENGTH)


def floor_to_gps(floor):
    return (floor[0] / FLOOR_LENGTH * GPS_LENGTH - FLOOR_ADD), (FLOOR_ADD - (floor[1] / FLOOR_LENGTH * GPS_LENGTH))


if __name__ == "__main__":

    robot = GotoRobot()
    robot.passive_wait(WAIT_TIME)
    obstacles = create_wall()
    path = robot.receive_path(gps_to_floor(robot.get_gps_position()))
    name = robot.robot.getName()
    additional_time = 0
    while path is None:
        robot.passive_wait(WAIT_TIME + additional_time)
        additional_time += 1
        path = robot.receive_path(gps_to_floor(robot.get_gps_position()))

    for loc in path:
        next_pos = floor_to_gps(loc)
        robot.goto(next_pos)
