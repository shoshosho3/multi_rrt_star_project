from math import pi
import random
import time
from controller import Robot
# import controllers.my_ground.my_ground as my_ground
import math
import sys

# Constants
TIME_STEP = 64
MAX_SPEED = 16
HALF_SPEED = 8
NULL_SPEED = 0
WHEEL_RADIUS = 0.031
AXLE_LENGTH = 0.271756

MY_NUMBER = 1.2335

# Device names
BUMPERS_NAME = ["bumper_left", "bumper_right"]
CLIFF_SENSORS_NAME = ["cliff_left", "cliff_front_left", "cliff_front_right", "cliff_right"]
LEDS_NAME = ["led_on", "led_play", "led_step"]


class CreateRobot:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())

        self.name = self.robot.getName()

        # Retrieve controller arguments
        args = self.robot.getCustomData()
        print(f'Arguments: {args}')

        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.time_step)

        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.time_step)

        self.compass2 = self.robot.get_compass_reading()

        print(math.readians(self.compass2))

        # Initialize devices
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(self.time_step)

        self.bumpers = [self.robot.getDevice(name) for name in BUMPERS_NAME]
        for bumper in self.bumpers:
            bumper.enable(self.time_step)

        self.cliff_sensors = [self.robot.getDevice(name) for name in CLIFF_SENSORS_NAME]
        for sensor in self.cliff_sensors:
            sensor.enable(self.time_step)

        self.leds = [self.robot.getDevice(name) for name in LEDS_NAME]

        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.angular_speed = (MAX_SPEED * WHEEL_RADIUS) / AXLE_LENGTH

    def step(self):
        if self.robot.step(self.time_step) == -1:
            exit(0)

    def is_collision(self, side):
        return self.bumpers[side].getValue() != 0.0

    def is_cliff(self, side):
        return self.cliff_sensors[side].getValue() < 100.0

    def is_virtual_wall(self):
        return self.receiver.getQueueLength() > 0

    def flush_receiver(self):
        while self.receiver.getQueueLength() > 0:
            self.receiver.nextPacket()

    def go_forward(self):
        self.left_motor.setVelocity(MAX_SPEED)
        self.right_motor.setVelocity(MAX_SPEED)

    def go_backward(self):
        self.left_motor.setVelocity(-HALF_SPEED)
        self.right_motor.setVelocity(-HALF_SPEED)

    def stop(self):
        self.left_motor.setVelocity(NULL_SPEED)
        self.right_motor.setVelocity(NULL_SPEED)

    def passive_wait(self, seconds):
        start_time = self.robot.getTime()
        while self.robot.getTime() < start_time + seconds:
            self.step()

    def turn(self, angle):
        self.stop()
        l_offset = self.left_position_sensor.getValue()
        r_offset = self.right_position_sensor.getValue()
        neg = -1.0 if angle < 0 else 1.0

        self.left_motor.setVelocity(neg * HALF_SPEED)
        self.right_motor.setVelocity(-neg * HALF_SPEED)

        orientation = 0
        while abs(orientation) < abs(angle):
            l = self.left_position_sensor.getValue() - l_offset
            r = self.right_position_sensor.getValue() - r_offset
            dl = l * WHEEL_RADIUS
            dr = r * WHEEL_RADIUS
            orientation = neg * (dl - dr) / AXLE_LENGTH
            self.step()

        self.stop()
        self.step()

    def turn_until(self, angle):
        self.stop()
        l_offset = self.left_position_sensor.getValue()
        r_offset = self.right_position_sensor.getValue()
        neg = -1.0 if angle < 0 else 1.0

        self.left_motor.setVelocity(neg * HALF_SPEED)
        self.right_motor.setVelocity(-neg * HALF_SPEED)

        compass_values = self.compass.getValues()
        print(f'compass_values: {compass_values}')

        orientation = 0

        while abs(orientation) < abs(angle):
            l = self.left_position_sensor.getValue() - l_offset
            r = self.right_position_sensor.getValue() - r_offset
            dl = l * WHEEL_RADIUS
            dr = r * WHEEL_RADIUS
            orientation = neg * (dl - dr) / AXLE_LENGTH
            self.step()

        self.stop()
        self.step()

    def turn_to(self, angle):
        current_angle = self.compass.getValues()[1]
        angle = angle - current_angle
        if angle > pi:
            angle -= 2 * pi
        elif angle < -pi:
            angle += 2 * pi
        self.turn(angle)

    def distance(self, target):
        current_position = self.gps.getValues()
        return math.sqrt((target[0] - current_position[0]) ** 2 + (target[1] - current_position[1]) ** 2)

    def angle_to(self, target):
        current_position = self.gps.getValues()
        return math.atan2(target[1] - current_position[1], target[0] - current_position[0])

    def angle_from(self, target):
        current_position = self.gps.getValues()
        return math.atan2(current_position[1] - target[1], current_position[0] - target[0])

    def run(self):
        print("Default controller of the iRobot Create robot started...")
        random.seed(time.time())

        # patch_centers = my_ground.patch_centers

        self.leds[0].set(1)
        self.passive_wait(0.5)

        printed = False

        prev_position = self.gps.getValues()

        self.turn_until(pi)

        self.turn_until(pi)

        while True:
            # getting left position sensor within the 512*512 image
            current_position = self.gps.getValues()

            self.turn(pi)
            break

            if self.name == "Create":
                next_position = (2.27, -0.5)
            else:
                next_position = (2, -1)

            if distance(next_position) < 0.05:
                if not printed:
                    print("Arrived at destination")
                    print(f'current_position: {current_position}')
                    printed = True
                self.stop()
            elif self.is_virtual_wall():
                print("Virtual wall detected")
                self.turn(pi)
            elif self.is_collision(0) or self.is_cliff(0):
                print("Left obstacle detected")
                self.go_backward()
                self.passive_wait(0.5)
                self.turn(pi * random.random())
            elif self.is_collision(1) or self.is_cliff(1) or self.is_cliff(2):
                print("Right obstacle detected")
                self.go_backward()
                self.passive_wait(0.5)
                self.turn(-pi * random.random())
            # else:
            #     self.go_forward()
            #     if self.angle_from(prev_position) < self.angle_to(next_position) - 0.1:
            #         self.left_motor.setVelocity(MAX_SPEED / 2)
            #     elif self.angle_from(prev_position) > self.angle_to(next_position) + 0.1:
            #         self.right_motor.setVelocity(MAX_SPEED / 2)
            #
            # self.flush_receiver()
            # self.step()


# Main
if __name__ == "__main__":
    robot = CreateRobot()
    robot.run()
