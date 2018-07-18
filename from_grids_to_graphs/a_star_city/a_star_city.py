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

# In[1]:
import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid
from planning import a_star

#get_ipython().run_line_magic('matplotlib', 'inline')


# In[2]:
plt.rcParams['figure.figsize'] = 12, 12


# You'll notice we've imported `create_grid`, and `a_star`.  These are
# functions you've implemented in previous exercises, you'll use them to create
# a map and find a path from a starting position to a goal position.
#
# To read the function signature and documentation execute `?` followed by the
# function name in a cell.  In the example below we'll check the documentation
# for `create_grid`.

# In[3]:

#get_ipython().run_line_magic('pinfo', 'create_grid')


# If you would like to alter these implementations, you can modify
# [grid.py](/edit/grid.py) and [planning.py](/edit/planning.py) in the current
# directory.

# In[4]:


# This is the same obstacle data from the previous lesson.
filename = 'a_star_city/colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)


# In[5]:


# Static drone altitude (meters)
drone_altitude = 5

# Minimum distance stay away from obstacle (meters)
safe_distance = 3


# In[6]:


# TODO: Use `create_grid` to create a grid configuration space of
# the obstacle data.
grid = create_grid(data, drone_altitude, safe_distance)


# In[7]:


# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, origin='lower') 

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


# Next you'll compute the path from a start location to a goal location using
# A*.

# Start and goal coordinates in *(north, east)*.

# In[8]:
start_ne = (25,  100)
goal_ne = (750., 370.)


# Write a heuristic function.

# In[9]:
def heuristic_func(position, goal_position):
    return np.linalg.norm(np.subtract(position, goal_position))


# Compute the lowest cost path with `a_star`.

# In[10]:
path, cost = a_star(grid, heuristic_func, start_ne, goal_ne)
print(len(path), cost)


# Let's plot the path!

# In[11]:
plt.imshow(grid, cmap='Greys', origin='lower')

# For the purposes of the visual the east coordinate lay along
# the x-axis and the north coordinates long the y-axis.
plt.plot(start_ne[1], start_ne[0], 'x')
plt.plot(goal_ne[1], goal_ne[0], 'x')

curr_pos = start_ne
actual_path = []
for action in path:
    curr_pos = (curr_pos[0] + action.delta[0], curr_pos[1] + action.delta[1])
    actual_path.append(curr_pos)

pp = np.array(actual_path)
plt.plot(pp[:, 1], pp[:, 0], 'g')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


# Pretty neat!  Unfortunately this path is impractical, the drone would be
# constantly stopping and going since each grid cell visited is synonymous with
# a waypoint.  Ideally, we'd like to only consider a new waypoint when the
# drone's direction changes.  Path pruning is one way to achieve this result.

# ### Path Pruning

# In[12]:
def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def bres(start, end):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    d = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    j = y1
    cells = []
    for i in range(x1, x2 + 1):
        coord = (j, i) if is_steep else (i, j)
        cells.append(coord)
        d -= abs(dy)
        if d < 0:
            j += ystep
            d += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        cells.reverse()
    return cells


# Complete the `prune_path` function below.  It should return a new path much
# shorter than the original.

# In[13]:


# We're using collinearity here, but you could use Bresenham as well!
def prune_path(path):
    pruned_path = [p for p in path]
    # TODO: prune the path!
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i + 1])
        p3 = point(pruned_path[i + 2])

        #bresen = bres(p1, p2)
        #return bresen
        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_check(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1
    return pruned_path

def prune_path_bresenham(path):
   """
   Use the Bresenham module to trim uneeded waypoints from path
   """
   from bresenham import bresenham
   pruned_path = [p for p in path]

   i = 0
   while i < len(pruned_path) - 2:
       p1 = pruned_path[i]
       p2 = pruned_path[i + 1]
       p3 = pruned_path[i + 2]
       
       # if the line between p1 and p2 doesn't hit an obstacle
       # remove the 2nd point.
       # The 3rd point now becomes the 2nd point
       # and the check is redone with a new third point
       # on the next iteration.
       br = list(bresenham(p1[0], p1[1], p3[0], p3[1]))
       #my_br = bres(p1, p3)

       if all((grid[pp] == 0) for pp in br):
           # Something subtle here but we can mutate
           # `pruned_path` freely because the length
           # of the list is checked on every iteration.
           pruned_path.remove(p2)
       else:
           i += 1

   return pruned_path

def prune_path_bres(path):
   """
   Use the Bresenham module to trim uneeded waypoints from path
   """
   pruned_path = [p for p in path]

   i = 0
   while i < len(pruned_path) - 2:
       p1 = pruned_path[i]
       p2 = pruned_path[i + 1]
       p3 = pruned_path[i + 2]
       
       # if the line between p1 and p2 doesn't hit an obstacle
       # remove the 2nd point.
       # The 3rd point now becomes the 2nd point
       # and the check is redone with a new third point
       # on the next iteration.
       #br = list(bresenham(p1[0], p1[1], p3[0], p3[1]))
       my_br = bres(p1, p3)

       if all((grid[pp] == 0) for pp in my_br):
           # Something subtle here but we can mutate
           # `pruned_path` freely because the length
           # of the list is checked on every iteration.
           pruned_path.remove(p2)
       else:
           i += 1

   return pruned_path

    


# Prune the path.

# In[ ]:
pruned_path = prune_path(actual_path)
print(len(pruned_path))

pruned_path_Bresenham = prune_path_bresenham(actual_path)
print(len(pruned_path_Bresenham))

pruned_path_B = prune_path_bres(actual_path)
print(len(pruned_path_B))


# In[ ]:
pruned_path
pruned_path_Bresenham
pruned_path_B


# Replot the path, it will be the same as before but the drone flight will be
# much smoother.

# In[ ]:
plt.imshow(grid, cmap='Greys', origin='lower')

plt.plot(start_ne[1], start_ne[0], 'x')
plt.plot(goal_ne[1], goal_ne[0], 'x')

pp = np.array(pruned_path)
ppB = np.array(pruned_path_Bresenham)
ppb = np.array(pruned_path_B)

plt.plot(pp[:, 1], pp[:, 0], 'b')
plt.scatter(pp[:, 1], pp[:, 0])

plt.plot(ppB[:, 1], ppB[:, 0], 'r')
plt.scatter(ppB[:, 1], ppB[:, 0])

plt.plot(ppb[:, 1], ppb[:, 0], 'g')
plt.scatter(ppb[:, 1], ppb[:, 0])

plt.xlabel('EAST')
plt.ylabel('NORTH')

plt.show()


# Now the waypoints symbolize a change in direction, much better!