import numpy as np 
from itertools import combinations
from pointhelper import *

def geo_trilaterate(mic_array, DOFs):
	mdata = list(zip(mic_array, DOFs))

	sols = []
	solvector = np.zeros(2)
	# iterate thru each combination of mics 
	for mdata1, mdata2 in combinations(mdata, 2):
		d1 = mdata1[1]
		d2 = mdata2[1]
		mic1_pos = mdata1[0]
		mic2_pos = mdata2[0]

		# find intersections
		sol1, sol2 = circle_intersection(mic1_pos, mic2_pos, d1, d2)
		
		sols.append([sol1, sol2])
		solvector += sol1/norm(sol1) + sol2/norm(sol2)

		solvector /= len(sols)

	true_sols = []
	# iterate thru sols, find the correct sol from pair
	# by comparing angle with solvector
	for sol1, sol2 in sols:
		# print('1: {}, 2: {}'.format(sol1, sol2))
		a1, a2 = angle(sol1, solvector), angle(sol2, solvector)
		if a1 < a2:
			true_sols.append(sol1)
		else:
			true_sols.append(sol2)

	# final centroid of all true_sols
	final_sol = np.zeros(2)
	for point in true_sols:
		final_sol += point
	final_sol /= len(true_sols)

	return final_sol

# p1, p2 are 2D numpy arrays, d1, d2 are scalar circle radii
# returns two solutions as numpy 2-arrays
def circle_intersection(p1, p2, d1, d2):
	l = p2 - p1								
	l_sq = (l**2).sum()						# distance between p1 and p2
	theta = l.dot([1,0]) / np.sqrt(l_sq)	# angle between l vec and x axis
	theta = np.arccos(theta)
	if l[1] < 0:
		theta = -theta						# account for 3rd, 4th quadrants

	cos_a1 = (d1**2 - d2**2 + l_sq) / (2*np.sqrt(l_sq)*d1)

	x = d1*cos_a1
	yp = d1*np.sqrt(1 - cos_a1**2)
	yn = -yp

	# transform into frame of p1 and p2
	# first rotate by theta, then translate origin to p
	sol1 = p1 + rotate(np.array((x,yp)), theta)
	sol2 = p1 + rotate(np.array((x,yn)), theta)

	return [sol1, sol2]




