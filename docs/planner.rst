Documentation
=============

planner.py
***********
Discrete motion planners for a holonomic robot. Robot moves in a static and flat environment and must find a path to goal.

RandomPlanner
-------------
.. autoclass:: planner.RandomPlanner
    :members: search, __init__

.. code-block:: python 

   >>> import planner as pln
   >>> world_state = [[0,0,1,0,0,0],
   ...                [0,0,1,0,0,0],
   ...                [0,0,0,0,1,0],
   ...                [0,0,0,0,1,0],
   ...                [0,0,1,1,1,0],
   ...                [0,0,0,0,0,0]]   
   >>> robot_pose = (2,0)
   >>> goal_pose = (5,5)
   >>> planner = pln.RandomPlanner(20)
   >>> planner.search(world_state, robot_pose, goal_pose)
   [(2, 0), (2, 1), (2, 2), (2, 3), (3, 3), (3, 2), (3, 1), 
   (4, 1), (5, 1), (5, 2), (5, 3), (5, 4), (5, 5)]

OptimalPlanner
---------------
.. autoclass:: planner.OptimalPlanner
    :members:
 
.. code-block:: python 

   >>> import planner as pln
   >>> world_state = [[0,0,1,0,0,0],
   ...		      [0,0,1,0,0,0],
   ...		      [0,0,0,0,1,0],
   ...		      [0,0,0,0,1,0],
   ...                [0,0,1,1,1,0],
   ...		      [0,0,0,0,0,0]]
   >>> robot_pose = (2,0)
   >>> goal_pose = (5,5)
   >>> planner = pln.OptimalPlanner()
   >>> planner.search(world_state, robot_pose, goal_pose)
   [(2, 0), (3, 0), (4, 0), (5, 0), (5, 1), (5, 2), (5, 3), 
   (5, 4), (5, 5)]
