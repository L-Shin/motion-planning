Algorithms and complexity
===========================

Randomized planner:
********************

Valid movements
----------------

The `RandomPlanner` attempts to find a path from `robot_pose` to `goal_pose` by randomly moving in `world_state`. A valid movement for the robot is between any two orthogonally adjacent cells. For example, if the robot is currently at cell `(2, 2)` then its possible movements are to cells `(3, 2)`, `(2, 3)`, `(1, 2)`, or `(2, 1)`. However, each of these moves is only valid if: 

#. The destination cell exists in `world_state`,

#. `world_state` contains a `0` entry at the destination,

#. AND the destination has not been visited in the last `sqrt(max_step_number)` steps, or all possible movements fail this condition.

Runtime of a single movement
-----------------------------
From a given cell, the random algorithm can collect and validate all adjacent cells in O(1) time. If some of these cells have been visited too recently (within `sqrt(max_step_number)` steps), these cells are removed from the list of potential moves unless this would leave no remaining possible moves. 

Total complexity
-----------------
During set-up the `visits` array is initialized with the same shape as `world_state` and if `world_state` is a list of lists, it is cast to a numpy array. Both of these operations require O(`|world_state|`) time. Therefore total runtime for the algorithm is O(`max_step_number + |world_state|`). The space complexity is O(`|world_state|`).

Optimal planner:
****************

Valid movements
---------------
The `OptimalPlanner` must satisfy the same requirements for valid movements between cells, except that there is no need to enforce the third condition. This is because any optimal path from `robot_pose` to `goal_pose` will not visit any cell more than once. The `visited` array enforces that no array is revisited.

World state graph
-----------------
`world_state` implicitly defines an undirected, unweighted graph on its navigable cells. Any one navigable cell has up to 4 adjacent cells, defined as the orthogonally adjacent cells in `world_state` that contain `0's`. Just as if the graph had an adjacency list, the adjacent cells in the `world_state` graph can be found in O(1) time. Therefore, the shortest path from `robot_pose` to `goal_pose` can be found in O(`|world_state|`) time using breadth first search. 

Total complexity
-----------------
During set-up the `visited` and `previous` arrays are initialized with the same shape as `world_state`, and if `world_state` is a list of lists it is cast to a numpy array. Therefore, runtime and space complexity for the algorithm are both O(`|world_state|`). 

Room for improvement
*********************
Assuming that `world_state` is supplied as a 2D numpy array, both the random and optimized algorithms could potentially have much smaller runtime in practice if the target path is much much smaller than the total size of `world_state`. Future implementations should consider using dictionaries/lists instead of arrays for `visits`, `previous`, and `visited`. Dictionaries/lists have constant time add/check operations and grow as needed. This would allow the runtime of both algorithms to reflect that they only explore a small part of the graph if possible. However, if the target path is not much shorter than `world_state`, these changes will decrease performance since the constant time operations for a dictionary with tuple keys are slower than those for a 2D array with tuple indices.
