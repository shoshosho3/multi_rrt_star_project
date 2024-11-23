from my_ground_functions import *


def main():
    # Set battery mode to True to run the battery mode simulation
    battery_mode = False

    # True - run series of simulations
    # False - run single simulation
    run_simulations_mode = True

    # True - save the results of the simulation
    # False - do not save the results of the simulation
    save_mode = False

    # parameters for simulation mode
    min_robots = 1
    max_robots = 5

    # for single simulation mode
    robot_number = 1
    seed = 42
    patch_number = 7

    print('Starting supervisor...')

    # Setup simulation environment
    robot = Supervisor()

    # Initialize devices and objects
    display = robot.getDevice(DISPLAY)
    emitter = robot.getDevice(EMITTER)

    if run_simulations_mode:
        if battery_mode:
            my_bots = run_simulations(robot, display, emitter, battery_mode, min_robots, max_robots,
                                      patch_numbers=[1, 2, 3, 4, 7, 10], save_mode=save_mode)
        else:
            my_bots = run_simulations(robot, display, emitter, battery_mode, min_robots, max_robots,
                                      save_mode=save_mode)

    else:
        my_bots = run_simulations(robot, display, emitter, battery_mode, robot_number, robot_number,
                                  [patch_number], [seed], save_mode=save_mode)

    terminal_message(emitter, my_bots)


# Run the main function
if __name__ == "__main__":
    main()
