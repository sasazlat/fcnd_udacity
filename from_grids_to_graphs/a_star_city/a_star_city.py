# coding: utf-8

# ## Finding Your Way In The City
#
# In this notebook you'll combine the work of previous exercises to calculate a
# minimal series of waypoints in order to get from a start location to a goal
# location.
#
# You'll reuse and modify your algorithms from:
#
# - A*
# - Configuration Space
# - Collinearity and/or Bresenham

# In[ ]:
import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid
from planning import a_star

#get_ipython().run_line_magic('matplotlib', 'inline')

#from bresenham import bresenham


# In[ ]:
plt.rcParams['figure.figsize'] = 12, 12


# You'll notice we've imported `create_grid`, and `a_star`.  These are
# functions you've implemented in previous exercises, and here you'll use them
# to create a map and find a path from a starting position to a goal position.
#
# To read the function signature and documentation execute `?` followed by the
# function name in a cell.  In the example below we'll check the documentation
# for `create_grid`.

# In[ ]:

#get_ipython().run_line_magic('pinfo', 'create_grid')


# If you would like to alter these implementations, you can modify
# [grid.py](/edit/grid.py) and [planning.py](/edit/planning.py) in the current
# directory.

# In[ ]:


# This is the same obstacle data from the previous lesson.
filename = 'a_star_city/colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)


# In[ ]:


# Static drone altitude (meters)
drone_altitude = 5

# Minimum distance stay away from obstacle (meters)
safe_distance = 3


# In[ ]:


# TODO: Use `create_grid` to create a grid configuration space of
# the obstacle data.
grid = create_grid(data, drone_altitude, safe_distance)


# In[ ]:


# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, origin='lower') 

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


# Next you'll compute the path from a start location to a goal location using
# A*.

# Start and goal coordinates in *(north, east)*.

# In[ ]:
start_ne = (25,  100)
goal_ne = (750., 370.)


# Write a heuristic function.

# In[ ]:
def heuristic_func(position, goal_position):
    # TODO: write a heuristic!
    x1,y1 = position
    x2,y2 = goal_position
    return np.absolute(np.absolute(x2-x1) + np.absolute(y2-y1))


# Compute the lowest cost path with `a_star`.

# In[ ]:


# TODO: use `a_star` to compute the lowest cost path
path, cost = a_star(grid, heuristic_func, start_ne, goal_ne)
print(len(path), cost)


# Let's plot the path!

# In[ ]:
plt.imshow(grid, cmap='Greys', origin='lower')

# For the purposes of the visual the east coordinate lay along
# the x-axis and the north coordinates long the y-axis.
plt.plot(start_ne[1], start_ne[0], 'x')
plt.plot(goal_ne[1], goal_ne[0], 'x')

pp = np.array(path)
#plt.plot(pp[:, 1], pp[:, 0], 'g')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


# Pretty neat!  Unfortunately this path is impractical, the drone would be
# constantly stopping and going since each grid cell visited is synonymous with
# a waypoint.  Ideally, we'd like to only consider a new waypoint when the
# drone's direction changes.  Path pruning is one way to achieve this result.

# ### Path Pruning

# In[ ]:
def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon


# Complete the `prune_path` function below.  It should return a new path much
# shorter than the original.

# In[ ]:
def prune_path(path):
    pruned_path = [p for p in path]
    # TODO: prune the path!
    return pruned_path


# Prune the path.

# In[ ]:
pruned_path = prune_path(path)
print(len(pruned_path))


# In[ ]:
pruned_path


# Replot the path, it will be the same as before but the drone flight will be
# much smoother.

# In[ ]:
plt.imshow(grid, cmap='Greys', origin='lower')

plt.plot(start_ne[1], start_ne[0], 'x')
plt.plot(goal_ne[1], goal_ne[0], 'x')

pp = np.array(pruned_path)
plt.plot(pp[:, 1], pp[:, 0], 'g')
plt.scatter(pp[:, 1], pp[:, 0])

plt.xlabel('EAST')
plt.ylabel('NORTH')

plt.show()


# Now the waypoints symbolize a change in direction, much better!

# [solution](/notebooks/A-Star-City-Solution.ipynb)
