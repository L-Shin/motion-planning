[Lauren Shin's discrete motion planners!](https://motion-planning.readthedocs.io/en/latest/)
========================================
The robot lives in a 2D grid environment in which every cell is either occupied or free. These motion planners allow the robot to navigate between any two free cells in its environment.

Code
-----

Two planners, random and optimal, are found in `planner.py`.

There are two sets of `unittest` tests. `test_Planner.py` tests the `search` methods in `planner.py`. `test_Utils.py` tests the non-public utilities in `planner.py`.

There is one file `timing.py` that tests runtime and path length for both search methods in `planner.py`.

Input files
-----------
The `input` folder contains a few NumPy array files containing cool discretized world states. `test_maze()` in `test_Planner.py` demonstrates how a planner can be used to navigate in a NumPy array file.

Documentation
--------------
You can find documentation and more information [here](https://motion-planning.readthedocs.io/en/latest/). 

