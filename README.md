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

## Contributing

This project is currently not open to external contributions. Thank you for your interest!

## License

[MIT](https://choosealicense.com/licenses/mit/)