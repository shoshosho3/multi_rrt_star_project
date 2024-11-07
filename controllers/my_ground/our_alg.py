import numpy as np
from tqdm import tqdm
from typing import List, Tuple
from consts import *
import warnings
from plots import create_plot
from wall_utils import Wall
from rrt_solver import NewRRTSolver


def full_round(x):
    """
    this is just the round function with an out of bounds check
    """

    x = round(x)
    if x < 0:
        x = 0
    if x >= FLOOR_LENGTH:
        x = FLOOR_LENGTH - 1
    return x


def run_solver(solver: NewRRTSolver):
    """
    run the solver for a given number of iterations and look for a legal allocation

    @param solver: a new rrt solver object
    @return: the allocation and the paths if found, otherwise None

    """

    # run the solver for a given number of iterations
    for _ in tqdm(range(MAX_ITERATIONS_OUR // 5)):
        solver.random_expend_tree()

    # run until the trees are connected or the max iterations are reached
    for _ in tqdm(range(MAX_ITERATIONS_OUR)):
        if solver.are_trees_connected():
            tqdm.write('connected')
            break
        solver.random_expend_tree()

    # check if the trees are connected, if not return None
    if not solver.are_trees_connected():
        print('no connection')
        return None, None

    # allocate the goals
    goals = solver.random_allocate_goals(10000)

    # check if there is no allocation, if so return None
    if goals[0] is None:
        print('no allocation')
        return None, None

    # return the allocation and the paths
    return goals


def is_coords_valid(coords, to_avoid):
    """
    check if the coordinates are valid
    :param coords: coordinates to check
    :param to_avoid: matrix of obstacles
    :return: true if the coordinates are valid, otherwise false
    """
    return (0 <= coords[0] < FLOOR_LENGTH and 0 <= coords[1] < FLOOR_LENGTH
            and not to_avoid[full_round(coords[0])][full_round(coords[1])])


def add_nodes(path, node):
    """
    add the nodes to the path
    :param path: path to add the nodes to
    :param node: node to add
    """
    while node is not None:
        path.append(tuple(node.coordinates))
        node = node.parent


def get_total_path(ways, dirt_locations, starts, solver):
    """
    get the total path for the bots
    :param ways: ways to get the path
    :param dirt_locations: dirt locations
    :param starts: start locations
    :param solver: solver object
    :return: the total path for the bots
    """

    # initialize the total path
    total_path = []

    for i in range(len(ways) - 1):

        # get the connection between the two ways
        connection = solver.tree_connections[tuple(sorted([ways[i], ways[i + 1]]))]

        # get the path
        path = []
        add_nodes(path, connection.node1)
        path.reverse()
        add_nodes(path, connection.node2)

        # check if the path is reversed
        if dirt_locations[ways[i + 1] - len(starts)] != path[-1]:
            path.reverse()

        # add the path to the total path
        if i == 0:
            total_path.extend(path)
        else:
            total_path.extend(path[1:])

    return total_path


def get_paths(bot_ways, dirt_locations, starts, solver):
    """
    get the paths for the bots
    :param bot_ways: the tree ids paths
    :param dirt_locations: dirt locations
    :param starts: start locations
    :param solver: rrt solver object
    :return: the paths for the bots by coordinates
    """
    paths = []
    for ways in bot_ways:
        total_path = get_total_path(ways, dirt_locations, starts, solver)
        paths.append(total_path)
    return paths


def run(dirt_locations: List[Tuple[int, int]], starts: List[tuple], walls: Wall):
    """
    run the algorithm to get the paths for the robots
    :param dirt_locations: dirt locations
    :param starts: start locations
    :param walls: wall object to get the obstacles
    :return: paths for the robots if found, otherwise empty list
    """

    warnings.filterwarnings("ignore")

    # Initialize floor and dirt locations
    to_avoid = walls.get_obstacle_matrix(AVOID_DISTANCE)
    obstacles = walls.get_obstacle_matrix(0)
    goal_sources = np.array(dirt_locations)
    bounds = np.array([[0, 0], [FLOOR_LENGTH, FLOOR_LENGTH]])

    # initializing the rrt solver
    solver = NewRRTSolver(np.array(starts), goal_sources, bounds, P, STEP_SIZE, is_coords_valid, to_avoid,
                          P_CONTRACTION, MIN_SPEED_ALG, MAX_SPEED_ALG, MAX_TURN_TIME,
                          COLLISION_DISTANCE)

    # getting the tree paths
    al, _ = run_solver(solver)
    if al is None:  # no legal allocation found
        return [[] for _ in range(len(starts))]
    final_tree_paths = [[key] + [p + len(al) for p in value] for key, value in enumerate(al)]
    print(f'final paths by tree ids: {final_tree_paths}')

    # getting the paths
    paths = get_paths(final_tree_paths, dirt_locations, starts, solver)

    # create plot
    create_plot(obstacles, dirt_locations, starts, paths, solver)

    return paths
