"""
test_Planner.py
==================================
Tests for RandomPlanner and OptimalPlanner.

These tests were created with the standard unittest library.

Run with `$ python test_Planner.py`.

Uncomment lines 134-137 to view a cool maze solution!

You must run with Python 2.7.*. Tests use NumPy and Matplotlib.


"""

import unittest
import numpy as np
import planner as pln
from matplotlib import pyplot as plt

class RandomNav(unittest.TestCase):
	def test_basic(self):
		print """\nTesting RANDOM planner on simple 
			 6x6 world...\n"""
		timeout = 100
		(world_state, robot_pose, goal_pose) = get_basic()
		print "timeout:", timeout, "\n"
		
		# Get planner and path.
		planner = pln.RandomPlanner(timeout)
		path = planner.search(world_state, 
				      robot_pose, 
				      goal_pose)
		# Preprocess world_state so can use planner utils
		world_state = pln._rectangularize(world_state)
		
		if path != None:
			print "Path found!"
			print "Path length:", len(path)
			
			validate_ends(self, path, robot_pose, 
				      goal_pose)
			validate_path(self, path, world_state)
		
		else: print "Planner timed out."
	
	def test_timeout(self):
		print "\nTesting RANDOM planner times out...\n"
		# Impossible to find path in less than 6 steps
		timeout = 6
		(world_state, robot_pose, goal_pose) = get_basic()
		print "timeout:", timeout, "\n"
		
		# Get planner and path.
		planner = pln.RandomPlanner(timeout)
		path = planner.search(world_state, 
				      robot_pose, 
				      goal_pose)
		
		self.assertEqual(path, None)
		print "Planner timed out."
	
	def test_blocked(self):
		print """\nTesting RANDOM planner on blocked 
			 world...\n"""
		# World has no path from robot_pose to goal_pose.
		timeout = 100
		(world_state, robot_pose, goal_pose) = get_blocked()
		print "timeout:", timeout, "\n"
		
		# Get planner and path.
		planner = pln.RandomPlanner(timeout)
		path = planner.search(world_state, 
				      robot_pose, 
				      goal_pose)
	
		self.assertEqual(path, None)
		print "Planner timed out."

class OptimalNav(unittest.TestCase):
	def test_basic(self):
		print """\nTesting OPTIMAL planner on simple 
			 6x6 world...\n"""
		(world_state, robot_pose, goal_pose) = get_basic()
		# Get planner and path.
		planner = pln.OptimalPlanner()
		path = planner.search(world_state, 
				      robot_pose, 
				      goal_pose)
		# Preprocess world_state so can use planner utils.
		world_state = pln._rectangularize(world_state)
		
		# Make sure path exists.
		self.assertNotEqual(path, None)
		print "\nPath found!"

		# Make sure path is optimal.
		self.assertEqual(len(path), 9)
		print "Path length is optimal:", len(path)
		
		# Validate path.
		validate_ends(self, path, robot_pose, goal_pose)
		validate_path(self, path, world_state)
	
	def test_blocked(self):
		print """\nTesting OPTIMAL planner on blocked 
			 world...\n"""
		(world_state, robot_pose, goal_pose) = get_blocked()
		# Get planner and path.
		planner = pln.OptimalPlanner()
		path = planner.search(world_state, 
				      robot_pose, 
				      goal_pose)

		self.assertEqual(path, None)
		print "\nNo path found."

	def test_maze(self):
		print """\nTesting OPTIMAL planner on ~200x200 
			 maze...\n"""
		(maze, start, end) = get_maze()
		# Get planner and path.
		planner = pln.OptimalPlanner()
		path = planner.search(maze, start, end)
		
		self.assertNotEqual(path, None)
		print "Maze solution found!"
		
		print "Path length:", len(path)
		
		# Validate path.
		validate_ends(self, path, start, end)
		validate_path(self, path, maze)
		
		# UNCOMMENT THE BELOW LINES TO DISPLAY PATH IN MAZE
		#for cell in path:
		#	maze[cell] = 5
		#plt.imshow(maze)
		#plt.show()
		

# UTILS

def get_basic():
	world_state = [[0,0,1,0,0,0],
		       [0,0,1,0,0,0],
		       [0,0,0,0,1,0],
		       [0,0,0,0,1,0],
		       [0,0,1,1,1,0],
		       [0,0,0,0,0,0]]
	robot_pose = (2,0)
	goal_pose = (5,5)
	print "robot_pose:", robot_pose
	print "goal_pose:", goal_pose
	return (world_state, robot_pose, goal_pose)

def get_blocked():
	world_state = [[0,0,1,0,0,0],
		       [0,0,1,1,1,0],
		       [0,0,0,0,1,0],
		       [0,0,0,0,1,0],
		       [0,0,1,1,1,0],
		       [0,0,1,0,0,0]]
	robot_pose = (2,0)
	goal_pose = (5,5)
	print "robot_pose:", robot_pose
	print "goal_pose:", goal_pose
	return (world_state, robot_pose, goal_pose)
def get_maze():
	fname = "input/maze_map.npy"
	arr = np.load(fname)
	# 1's and 0's are switched in input
	arr = (arr + np.ones(arr.shape)) % 2
	start = (38, 1)
	end = (2, 122)
	return (arr, start, end)
def validate_ends(test, path, robot_pose, goal_pose):
	test.assertEqual(path[0], robot_pose)
	print "Path starts at robot_pose."
	
	test.assertEqual(path[-1], goal_pose)
	print "Path ends at goal_pose."


def validate_path(test, path, world_state):
	adj = [(1,0),(0,1),(-1,0),(0,-1)]
	prev = path[0]
	for cell in path[1:]:
		(px, py) = prev
		(x, y) = cell
		# Every cell in path must be valid.
		test.assertTrue(pln._navigable_in(cell, world_state))
		# Two adjacent cells in path must be orthogonally
		# adjacent in world_state.
		test.assertTrue((x-px,y-py) in adj)
		prev = cell
	print "Path contains valid orthogonal cells."


if __name__ == '__main__':
	unittest.main()
