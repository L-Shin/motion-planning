"""
planner.py
===========
Discrete motion planners for a holonomic robot. Robot moves in a static and
flat environment and must find a path to goal.  
"""

import numpy as np

class RandomPlanner:
	def __init__(self, max_step_number):
		"""
		Initialize `RandomPlanner`.

		Parameters
		----------
		max_step_number : int
			Maximum number of steps the planner can take before 
			timing out and returning `None`. Planner also refuses
			to revisit a cell that was visited in the last
			`sqrt(max_step_number)` steps unless this is the only
			available option.
		
		"""
		self.max_step_number = max_step_number

	def search(self, world_state, robot_pose, goal_pose):
		"""
		Find and return a path from current pose to goal pose by 
		randomly moving in the environment.
		
		Parameters
		----------
		world_state : numpy.ndarray or list
			A 2D numpy array or 2D list of lists containing 0's
			and 1's representing the environment. The value 0 
			indicates a navigable space and the value 1 
			indicates an occupied/obstructed space.
		robot_pose : tuple
			A tuple of two indices (x, y) which represent the 
			current pose of the robot in `world_state`.
		goal_pose : tuple
			A tuple of two indices (x, y) which represent the
			goal pose of the robot in `world_state`.
		
		Returns
		--------
		list 
			A list of tuples (x, y) which represent a path 
			from `robot_pose` to `goal_pose` in `world_state`, or 
			`None` if search times out.
		"""
		# Preprocess `world_state`.
		world_state = _rectangularize(world_state)
	
		# Check that `robot_pose` and `goal_pose` are               
		# valid and navigable.
		if not (_navigable_in(robot_pose, world_state) and 
			_navigable_in(goal_pose, world_state)):
			return None 
			
		# Set up an array `visits` to keep track of the most recent 
		# visit to each cell in `world_state`. If most recent 
		# visit was more than `forgetfulness` time steps ago, 
		# cell is considered unvisited. All cells are initially 
		# unvisited except `robot_pose`.
		forgetfulness = int(np.sqrt(self.max_step_number))
		visits = np.full(shape = world_state.shape, 
				 fill_value = -forgetfulness)
		visits[robot_pose] = 0 
		
		# Initialize robot path.
		path = [robot_pose]
		
		# Begin traveling. Planner must find solution in LESS 
		# THAN `max_step_number` steps, or else it returns `None`.
		for t in range(1, self.max_step_number):
			# Check whether path is complete.
			if robot_pose == goal_pose:
				return path

			# Collect all adjacent, navigable neighboring cells.
			neighbors = _get_neighbors(robot_pose, world_state)
			if len(neighbors) == 0: 
				return None
			
			# Impose condition that next cell has not 
			# been visited too recently.
			unvisited = [cell for cell in neighbors if 
				     (t - visits[cell] > forgetfulness)]
			# If all cells have been visited too recently, 
			# must ignore this condition.
			if len(unvisited) == 0:
				unvisited = neighbors

			# Choose next cell uniformly at random.
			rand_int = np.random.randint(len(unvisited))
			robot_pose = unvisited[rand_int]
			# Record visit to new location.
			visits[robot_pose] = t
			path.append(robot_pose)		
		
		# No path found in `max_step_number` steps.
		return None

class OptimalPlanner:
	def search(self, world_state, robot_pose, goal_pose):
		"""
		Find and return shortest path in `world_state` from 
		current pose to goal pose, avoiding obstacles.

		Parameters
		----------
		world_state : numpy.ndarray or list
			A 2D numpy array or 2D list of lists containing 0's
			and 1's representing the environment. The value 0 
			indicates a navigable space and the value 1 
			indicates an occupied/obstructed space.
		robot_pose : tuple
			A tuple of two indices (x, y) which represent the 
			current pose of the robot in `world_state`.
		goal_pose : tuple
			A tuple of two indices (x, y) which represent the
			goal pose of the robot in `world_state`.
		
		Returns
		-------
		list
			A list of tuples (x, y) which represent a path
			from `robot_pose` to `goal_pose`, or `None` if
			no such path exists.
		"""
		# Preprocess `world_state`.
		world_state = _rectangularize(world_state)
		world_shape = world_state.shape
		
		# Check that `robot_pose` and `goal_pose` are valid, 
		# navigable, and distinct.
		if not (_navigable_in(robot_pose, world_state) and 
			_navigable_in(goal_pose, world_state)): 
			return None 
		if robot_pose == goal_pose: return [robot_pose]
		
		# Initialize arrays used during BFS in `world_state`.
		# 	`previous[cell]` stores the tuple location 
		#       directly preceeding `cell` in the shortest path 
		#       from `robot_pose` to `cell`.
		# 
		#	`visited[cell]` stores whether cell has been 
		#	processed yet in the BFS.
		previous = np.empty(world_shape, dtype=(int,2))		
		visited = np.full(world_shape, fill_value=False) 
		visited[robot_pose] = True
		
		# Initialize queue of locations waiting to be processed.
		q = _Queue()
		q.enqueue(robot_pose)

		# Helper function visits neighbor from cell.
		def visit(neighbor, cell): 
			visited[neighbor] = True
			previous[neighbor] = cell
			q.enqueue(neighbor)
		
		# Begin breadth first search.
		while not q.is_empty():
			cell = q.dequeue()
			# Collect all adjacent, navigable neighboring cells.
			neighbors = _get_neighbors(cell, world_state)
			# Visit all unvisited neighbors.
			for neighbor in neighbors:
				if not visited[neighbor]:
					visit(neighbor, cell)
					# Check whether search is complete.
					if neighbor == goal_pose:
						return _build_path(
						 	    robot_pose, 
							    goal_pose,
							    previous)
		# `goal_pose` not reachable from `robot_pose`.
		return None
							
					
# UTILS

def _rectangularize(world_state):
	"""
	Return 2D numpy array containing rows of world_state padded so array 
	is rectangular. If world_state is already a 2D numpy array, just 
	return it and do nothing.
	"""
	# Check if `world_state` is already a 2D array 
	# (not an array of lists).
	if type(world_state) is np.ndarray: 
		if len(world_state.shape) == 2:
			return world_state
		else:
			# world_state is an array of lists. Convert to 
			# list of lists for rectangularization
			world_state = world_state.tolist()
	# Set width (of future 2D array) to be width of longest row.
	width = max([len(row) for row in world_state])
	# Pad each row with ones (cells added are not navigable space).
	for row in world_state:
		row.extend([1] * (width - len(row)))
	# Cast as 2D numpy array.
	return np.array(world_state)

def _navigable_in(location, world_state):
	"""
	Check that location is valid index in world_state
	and location is navigable.
	"""
	(x, y) = location
	(x_dim, y_dim) = world_state.shape
	x_valid = (0 <= x) and (x < x_dim)
	y_valid = (0 <= y) and (y < y_dim)
	# note that navigability is only checked if location is valid
	return x_valid and y_valid and world_state[location] == 0

def _get_neighbors(robot_pose, world_state):
	"""
	From tuple location `robot_pose` return tuple of all valid 
	orthogonally adjacent cells that are navigable in `world_state`.
	"""
	(x, y) = robot_pose
	neighbors = [(x + 1, y), (x, y + 1), 
			  (x - 1, y), (x, y - 1)]
	neighbors = [neighbor for neighbor in neighbors if 
		     _navigable_in(neighbor, world_state)]
	return neighbors

def _build_path(robot_pose, goal_pose, previous):
	"""
	Build path backwards from `goal_pose` to `robot_pose` 
	using the `previous` array, then return the path flipped.	
	"""
	path = [goal_pose]
	cell = goal_pose
	while cell != robot_pose:
		# `previous[cell]` returns list, so must convert to tuple.
		cell = tuple(previous[cell])
		path.append(cell)
	# Flip the path.
	path.reverse()
	return path
	
class _Queue:
	def __init__(self, capacity=4):
		"""
		Standard queue implementation for storing `(int,int)` tuples 
		with O(1) time enqueue and dequeue operations.
		"""
		self.queue = np.empty(capacity, dtype=(int,2))
		self.size = 0
		self.head = 0
		self.tail = 0
	
	def is_empty(self):
		return self.size == 0
	def _capacity(self):
		return self.queue.shape[0]
	def enqueue(self, item):
		if self.size == self._capacity():
			# Need more space in underlying array.
			self._resize(2 * self.size)
		# Store new item at tail.
		self.queue[self.tail] = item
		# Increment tail pointer, wrapping around end of array 
		# if necessary.
		self.tail = (self.tail + 1) % self._capacity()
		self.size += 1

	def dequeue(self):
		if self.size < self._capacity() / 4:
			# Underlying array is too big.
			self._resize(self._capacity() / 2)
		# Retrieve item at head of queue.
		item = tuple(self.queue[self.head])
		
		self.size -= 1
		# Increment head pointer, wrapping around end of array
		# if necessary.
		self.head = (self.head + 1) % self._capacity()
		return item
	
	def _resize(self, capacity):
		"""
		First roll `self.queue` head to index 0 of underlying array
		and then resize `self.queue`. 
		"""
		# This requires moving head `2*self.head` places backwards 
		# since roll flattens tuples in array.
		
		new_shape = (capacity,2) 
		self.queue = np.resize(np.roll(self.queue, -2*self.head), 
				       new_shape)
		self.head = 0
		self.tail = self.size 

