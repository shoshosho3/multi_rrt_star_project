# Multi-Agent Multi-Directional Cooperative RRT* With Webots Simulator

This project implements a Multi-Agent Multi-Directional Cooperative RRT* (Rapidly-exploring Random Tree Star) algorithm using the Webots simulator. The framework facilitates robot cooperation in navigating a shared environment.

### Key Directories and Files

#### **controllers**
- `goto_cont/`: Contains `goto_cont.py`, the controller code for robot movement.
- `my_ground/`: Contains `my_ground.py`, the supervisor controller code, responsible for managing the simulation parameters and running the simulations.

#### **worlds**
- `create4.wbt`: The main Webots world file, which should be loaded to run the simulations.

#### **Jupyter Notebooks**
- `data_analysis.ipynb`: Provides data analysis of the simulation results.
- `data_analysis_battery_mode.ipynb`: Focuses on analyzing simulations in battery mode.
- `new_alg_code.ipynb`: Implements and tests the new Multi-Agent Multi-Directional Cooperative RRT* algorithm.

## Simulation Overview

### Supervisor Parameters
The main simulation logic is managed by `my_ground.py`, which offers several configurable parameters:

```python
battery_mode = False  # Enable battery mode simulation
run_simulations_mode = True  # Enable series of simulations
save_mode = False  # Save simulation results

# Parameters for simulation mode
min_robots = 1  # Minimum number of robots
max_robots = 5  # Maximum number of robots

# Parameters for single simulation mode
robot_number = 1  # Number of robots
seed = 42  # Random seed
patch_number = 7  # Patch number
```

Modify these parameters to customize the simulation’s behavior, including the number of robots, whether to run in battery mode, and saving the simulation results.

### Prerequisites

	•	Webots simulator installed on your system.
	•	Python 3.x for running controllers and data analysis notebooks.
	•	Required Python libraries: numpy, matplotlib, jupyter, pandas, controller (Webots)

### Running the Code

	1.	Open Webots and load the create4.wbt world file from /worlds.
	2.	Click the play button to start the simulation.

## All files and directories

```bash
/controllers # This directory contains the controllers of the supervisor and the robots
  /goto_cont # This directory contains the controller of the robots
    goto_cont.py # This is the main file of the robot controller receiving the message from the supervisor 
                 # and running the robot from point to point
    goto_robot.py # This is the file that contains the class of the robot, controlling its parameters, movement,
                  # message receiving, battery handling, and other functions
    reg_rrt_star.py # This is the file that contains the class of the regular RRT* algorithm, which is used to find the 
                    # path to charger and back when battery mode is enabled
    unit_transformation.py # This is the file that contains functions to covert floor units to robot coordinates 
                           # and vice versa
    consts.py # This is the file that contains the constants used in the robot controller
  /my_ground
    my_ground.py # This is the main file of the supervisor controller, which is responsible for managing the simulation
                 # parameters, running the simulations, running our algorithm, and sending the path to the robots
    image_editor.py # a file for generating the dirt imagej
    my_ground_functions.py # This is the file that contains the main functions used in the supervisor controller
    robot_utils.py # This is the file that contains a class for the robot object, which is used to store the robot's 
                   # parameters and functions, setting its initial position, and getting the robot's current position
    wall_utils.py # This is the file that contains the functions used to create the walls in the simulation
    consts.py # This is the file that contains the constants used in the supervisor controller
    our_alg.py # This is the file that contains the class of our algorithm, which is used to find the path for the robots
    rrt_solver.py # This is the file that contains the functions and our multi-agent multi-directional cooperative RRT* 
                  # algorithm
    sampling_classes.py # This is the file that contains the classes used for sampling the points in our multi-agent 
                        # multi-directional cooperative RRT* algorithm
    tree_classes.py # This is the file that contains the classes used for the tree in our multi-agent multi-directional 
                    # cooperative RRT* algorithm
    segment.py # This is the file that contains the class used for the segment in our multi-agent multi-directional 
               # cooperative RRT* algorithm
    random_position.py # This is the file that contains the function used to generate a random position in the simulation
    plots.py # This is the file that contains the functions used to plot tree results and the path of the robots
    /dust_images # Dust images used in the simulation
      dust.jpg # The original image of the dirt
      dust_resized_circ_20.jpg # The resized image of the dirt with circles of radius 20 
    /results_csvs # results
      results.csv # The results of the simulation for regular mode
      battery_results.csv # The results of the simulation for battery mode
/plots # plots generated in the data analysis
/worlds # the world file to be loaded in Webots
  create4.wbt # the main world file
/protos # proto file
data_analysis.ipynb # a jupyter notebook for data analysis
data_analysis_battery_mode.ipynb # a jupyter notebook for data analysis in battery mode
new_alg_code.ipynb # a jupyter notebook for implementing and testing the multi-agent multi-directional
                   # cooperative RRT* algorithm
```

## Contributing

This project is currently not open to external contributions. Thank you for your interest!

## License

[MIT](https://choosealicense.com/licenses/mit/)