import numpy as np 

# returns distance between 2 2D points 
# p1 and p2 should be np arrays
def distance(p1, p2):
	return norm(p1 - p2)

# returns norm of vector v
def norm(v):
	vsq = v.dot(v).sum()
	return np.sqrt(vsq)

# abs angle between two 2D vectors
# output between 0 and pi
def angle(v1, v2, deg=False):
	theta = v1.dot(v2) / (norm(v1) * norm(v2))
	theta = np.arccos(theta)
	if deg:
		theta = theta * 180 / np.pi
	return theta

# rotate a point about origin by theta
def rotate(point, theta):
	c, s = np.cos(theta), np.sin(theta)
	R = np.array([[c, -s], [s, c]])
	return np.matmul(R, point.transpose())

# parametrically creates a set of points on unit circle
def create_unit_circle(num_points=100):
	circlex = [np.cos(t) for t in np.linspace(0, 2*np.pi, num_points+1)]
	circley = [np.sin(t) for t in np.linspace(0, 2*np.pi, num_points+1)]
	del circlex[-1]
	del circley[-1]
	return np.array(circlex), np.array(circley)

# create circle from center point and radius
# center can be list
def create_circle(center, radius, num_points=100):
	cx, cy = create_unit_circle(num_points)
	x = np.full_like(cx, center[0]) + radius*cx
	y = np.full_like(cy, center[1]) + radius*cy
	return x, y