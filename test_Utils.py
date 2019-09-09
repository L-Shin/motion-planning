"""
test_Utils.py
=============================
Tests for utils in planner.py

Run with `$ python test_Utils.py`

You must run with Python 2.7.*. Tests use NumPy.
"""

import unittest
import numpy as np
import planner as pln

class Rectangle(unittest.TestCase):
	def test_array(self):
		print """\nTesting whether rectangularize leaves a
			 2D numpy array untouched..."""
		a = np.array([[1,2],[3,4]])
		a_prime = pln._rectangularize(a)
		self.assertEqual(type(a_prime), np.ndarray)
		self.assertEqual(len(a.shape), 2)
		self.assertTrue(np.array_equal(a_prime, a))
		print "Success!"
	def test_2Dlist(self):
		print """\nTesting whether rectangularize correctly
			 pads a list of lists and casts to 2D numpy
			 array..."""
		l = [[1,2],[3,4,5]]
		a = np.array([[1,2,1],[3,4,5]])
		a_prime = pln._rectangularize(l)
		self.assertTrue(np.array_equal(a_prime, a))
		print "Success!"
	def test_array_of_lists(self):
		print """\nTesting whether rectangularize correctly 
			 pads a 1D numpy array of lists and casts to
			 2D numpy array..."""
		l = [[1,2],[3,4,5]]
		a = np.array([[1,2,1],[3,4,5]])
		a_prime = pln._rectangularize(l)
		self.assertTrue(np.array_equal(a_prime, a))
		print "Success!"
class Navigable(unittest.TestCase):
	def test_x(self):
		print """\nTesting whether navigable_in rejects invalid
			 x-coordinate..."""
		world_state = np.array([[0,0,0],[0,1,0]])
		self.assertFalse(pln._navigable_in((-1,0), world_state))
		self.assertFalse(pln._navigable_in((2,0), world_state))
		print "Success!"
	def test_y(self):
		print """\nTesting whether navigable_in rejects invalid
			 y-coordinate..."""
		world_state = np.array([[0,0,0],[0,1,0]])
		self.assertFalse(pln._navigable_in((0,-1), world_state))
		self.assertFalse(pln._navigable_in((0,3), world_state))	
		print "Success!"
	def test_nav(self):
		print """\nTesting whether navigable_in successfully
			 accepts/rejects based on whether the location
			 in world_state is a 1 or 0..."""
		world_state = np.array([[0,0,0],[0,1,0]])
		self.assertTrue(pln._navigable_in((0,0), world_state))
		self.assertFalse(pln._navigable_in((1,1), world_state))
		print "Success!"
class Neighbors(unittest.TestCase):	
	def test_none(self):
		print """\nTesting whether get_neighbors returns an 
			 empty list if the location has no 
			 neighbors..."""
		world_state = np.array([[0,1,0],[1,0,0]])
		self.assertEqual(len(pln._get_neighbors((0,0), 
				 world_state)), 0)
		print "Success!"
	def test_some(self):
		print """\nTesting whether get_neighbors correctly
			 selects navigable, orthogonally adjacent
			 neighbors..."""
		world_state = np.array([[0,1,0],[0,1,0],[0,0,0]])
		neighbors = pln._get_neighbors((1,0), world_state)
		self.assertEqual(len(neighbors), 2)
		self.assertTrue((0,0) in neighbors)
		self.assertTrue((2,0) in neighbors)
		print "Success!"
class TestQueue(unittest.TestCase):
	def test(self):
		print "\nTesting basic queue functionality..."
		q = pln._Queue()
		self.assertTrue(q.is_empty())
		q.enqueue((1,2))
		q.enqueue((3,4))
		q.enqueue((5,6))
		q.enqueue((7,8))
		q.enqueue((9,10))
		self.assertEqual(q.dequeue(), (1,2))
		self.assertEqual(q.dequeue(), (3,4))
		self.assertEqual(q.dequeue(), (5,6))
		q.enqueue((11, 12))
		self.assertEqual(q.dequeue(), (7,8))
		self.assertEqual(q.dequeue(), (9,10))
		self.assertFalse(q.is_empty())
		self.assertEqual(q.dequeue(), (11,12))
		self.assertTrue(q.is_empty())
		print "Success!"	


if __name__ == '__main__':
	unittest.main()

