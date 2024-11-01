from controller import Supervisor
import random
from consts import *
import numpy as np
import our_alg

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

def floor_to_gps(floor):
    return (floor[0] / FLOOR_LENGTH * GPS_LENGTH - FLOOR_ADD), (FLOOR_ADD - (floor[1] / FLOOR_LENGTH * GPS_LENGTH))

def robot_position(robot, obstacles):
    x = random.randint(50, 463)
    y = random.randint(50, 463)
    while obstacles[x][y] == HAS_OBSTACLE:
        x = random.randint(50, 463)
        y = random.randint(50, 463)
    x, y = floor_to_gps((x, y))
    robot.getField("translation").setSFVec3f([x, y, 0.044])


def battery_handle(robot, prev_bat):
    battery = robot.batterySensorGetValue()
    if int(battery / 13478.0 * 100) == prev_bat - 1:
        prev_bat -= 1
        print("Battery: " + str(int(battery / 13478.0 * 100)) + "%")
    return prev_bat


def create_wall(delta):
    obstacles_matrix = np.zeros((FLOOR_LENGTH, FLOOR_LENGTH))
    for x_position in range(WALL_X_LEFT - delta, WALL_X_RIGHT + delta):
        for y_position in list(range(0, WALL_Y_UP + delta)) + list(range(WALL_Y_DOWN - delta, FLOOR_LENGTH)):
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


def get_robot_names(supervisor):
    robot_names = []
    root_node = supervisor.getRoot()  # Access the root node
    children_field = root_node.getField("children")  # Get the children field of the root node
    num_children = children_field.getCount()  # Get the number of child nodes

    for i in range(num_children):
        node = children_field.getMFNode(i)  # Get each child node
        if node.getTypeName() == "Robot":  # Check if the node type is a Robot
            robot_names.append(node.getDef())  # Get the DEF name (robot name)

    return robot_names

def gps_to_floor(gps):
    return int((gps[0] + FLOOR_ADD) / GPS_LENGTH * FLOOR_LENGTH), int((FLOOR_ADD - gps[1]) / GPS_LENGTH * FLOOR_LENGTH)


# Main function
def main():
    print('starting')
    obstacles = create_wall(DIRT_DELTA)
    # Initialize Webots
    robot = Supervisor()

    # Get a handle to the ground display device
    display = robot.getDevice("ground_display")
    emitter = robot.getDevice("emitter")

    # Get the translation field of the robot
    my_bots = [robot.getFromDef("IROBOT_CREATE"), robot.getFromDef("IROBOT_CREATE_2")]
    # names = get_robot_names(robot)
    # my_bots = [robot.getFromDef(name) for name in names]
    for mybot in my_bots:
        robot_position(mybot, obstacles)
    translation_fields = [mybot.getField("translation") for mybot in my_bots]
    starts = [gps_to_floor(tuple(translation_field.getSFVec3f()[0:2])) for translation_field in translation_fields]
    print(f'starts: {starts}')
    print(f'translation_fields: {translation_fields}')

    patch_centers = create_dust(display, obstacles)
    paths = our_alg.run(patch_centers, starts, create_wall(0))

    message = str(paths).encode('utf-8')  # Encode list as bytes
    emitter.send(message)  # Send to robots

    # Set pen transparency to remove dust
    display.setAlpha(TRANSPARENCY)

    sim_loop(robot, display, translation_fields)

    robot.cleanup()


# Run the main function
if __name__ == "__main__":
    main()
