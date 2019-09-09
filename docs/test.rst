Testing
=======

.. automodule:: test_Planner
.. automodule:: test_Utils

Performance
************

.. automodule:: timing



Below is an example output from `timing.py`.

For the 200x200 maze, the random search failed to find a valid path through the maze. This is because of the nature of a maze: there are very few valid paths. Random movement is not a good strategy in a maze. 

For the simple 6x6 world, you can see that as `max_step_number` increases, the planner is more likely to return a valid path but the path increases in expected length. The average length of randomly generated paths all were larger than the length of the optimal path, but the random planner had faster average runtimes than the optimal planner.

~200x200 maze: random search
----------------------------

========== =========== ==============================
Timeout    Runtime (s) # of successful searches (/10)
========== =========== ==============================
 4000       0.0538      0
 8000       0.101       0
 16000      0.206       0
 32000      0.410       0
========== =========== ==============================

~200x200 maze: optimal search
-----------------------------

=========== =============
Runtime (s) Path length
=========== =============
 0.427       2244
=========== =============

6x6 basic: random search
-------------------------

======== ============  ==============================  ===========
Timeout  Runtime (ms)  # of successful searches (/10)  Path length
======== ============  ==============================  ===========
 10       0.148         0                               n/a
 20       0.236         3                               17.7
 40       0.320         7                               20.7
 80       0.304         10                              25.0
======== ============  ==============================  ===========

6x6 basic: optimal search
-------------------------

============ ============
Runtime (ms) Path length
============ ============ 
 0.860        9
============ ============



