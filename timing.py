"""
timing.py
==========
Compare performance of RandomPlanner and OptimalPlanner in planner.py.
"""

import time
import planner as pln
import numpy as np
import test_Planner
import statistics as stats

def random(arr, start, end, timeout):
	"""
	Run random planner 10 times and average the length of resulting
	path and runtime.
	"""
	print "   Timeout:", timeout, "steps"
	planner = pln.RandomPlanner(timeout)
	times = []
	lengths = []
	for _ in range(10):
		t = time.time()
		path = planner.search(arr, start, end)
		t = time.time() - t
		if path != None:
			lengths.append(len(path))
		times.append(t)
	print "   Avg runtime:", stats.mean(times), "s"
	print "  ", len(lengths), "of 10 attempts found solution"
	if len(lengths) > 0:
		print "   Avg path length:", stats.mean(lengths)
	print "\n" 
	

def optimal(arr, start, end):
	"""
	Run optimal planner and output length of path and runtime.
	"""
	planner = pln.OptimalPlanner()
	t = time.time()
	path = planner.search(arr, start, end)
	t = time.time() - t
	print "   Runtime:", t, "s"
	if path == None:
		print "   No path found.\n"
		return None
	else: print "   Path length:", len(path)
	print "\n"
	return len(path), t

if __name__ == '__main__':
	# Test performance on maze input.
	(arr, start, end) = test_Planner.get_maze()

	print "RANDOM search for 200x200 maze:\n"
	for timeout in [4000,8000,16000,32000]:
		random(arr, start, end, timeout)		

	print "OPTIMAL search for 200x200 maze:\n"
	optimal(arr, start, end)
	
	# Test performance on simple input.
	(arr, start, end) = test_Planner.get_basic()

	print "RANDOM search in 6x6 world:\n"
	for timeout in [10,20,40,80]:
		random(arr, start, end, timeout)

	print "OPTIMAL search in 6x6 world:\n"
	optimal(arr, start, end)
