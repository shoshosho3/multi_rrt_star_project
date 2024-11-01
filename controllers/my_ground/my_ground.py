from controller import Supervisor
import random
from consts import *
import numpy as np

patch_centers = []


def create_dust(display, obstacles):
    patches = []
    background = display.imageLoad(f"dust_resized_circ_20.png")
    for i in range(PATCH_NUMBER):
        x = random.randint(50, 463)
        y = random.randint(50, 463)
        while obstacles[x][y] == HAS_OBSTACLE:
            x = random.randint(50, 463)
            y = random.randint(50, 463)
        patches.append((x, y))
        display.imagePaste(background, x - 10, y - 10, False)
    return patches


def battery_handle(robot, prev_bat):
    battery = robot.batterySensorGetValue()
    if int(battery / 13478.0 * 100) == prev_bat - 1:
        prev_bat -= 1
        print("Battery: " + str(int(battery / 13478.0 * 100)) + "%")
    return prev_bat


def create_wall():
    obstacles_matrix = np.zeros((FLOOR_LENGTH, FLOOR_LENGTH))
    for x_position in range(WALL_X_LEFT - DIRT_DELTA, WALL_X_RIGHT + DIRT_DELTA):
        for y_position in list(range(0, WALL_Y_UP + DIRT_DELTA)) + list(range(WALL_Y_DOWN - DIRT_DELTA, FLOOR_LENGTH)):
            obstacles_matrix[x_position][y_position] = HAS_OBSTACLE
    return obstacles_matrix


def sim_loop(robot, display, translation_field):

    while robot.step(TIME_STEP) != TERMINATE_TIME_STEP:

        translations = [translation_field.getSFVec3f() for translation_field in translation_field]

        for translation in translations:

            # Calculate the position on the display and draw an oval
            x = int(FLOOR_LENGTH * (translation[0] + GROUND_X / 2) / GROUND_X)
            y = int(FLOOR_LENGTH * (-translation[1] + GROUND_Y / 2) / GROUND_Y)
            display.fillOval(x, y, 2 * Radius, 2 * Radius)


# Main function
def main():
    obstacles = create_wall()
    # Initialize Webots
    robot = Supervisor()
    robot.batterySensorEnable(1)

    # Get a handle to the ground display device
    display = robot.getDevice("ground_display")
    emitter = robot.getDevice("emitter")

    # Get the translation field of the robot
    # my_bots = [robot.getFromDef("IROBOT_CREATE"), robot.getFromDef("IROBOT_CREATE_2")]
    my_bots = [robot.getFromDef("IROBOT_CREATE")]
    translation_field = [mybot.getField("translation") for mybot in my_bots]

    patch_centers = create_dust(display, obstacles)
    message = str(patch_centers).encode('utf-8')  # Encode list as bytes
    emitter.send(message)  # Send to robots

    # Set pen transparency to remove dust
    display.setAlpha(TRANSPARENCY)

    sim_loop(robot, display, translation_field)

    robot.cleanup()


# Run the main function
if __name__ == "__main__":
    main()
