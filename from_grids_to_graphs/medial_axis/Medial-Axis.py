
# coding: utf-8

# ## Medial Axis
#

# In[1]:

import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid
from skimage.morphology import medial_axis
from skimage.util import invert
from planning import a_star
#get_ipython().run_line_magic('matplotlib', 'inline')


# In[2]:

plt.rcParams['figure.figsize'] = 12, 12


# In[3]:


# This is the same obstacle data from the previous lesson.
filename = 'medial_axis/colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)


# Starting and goal positions in *(north, east)*.

# In[4]:

start_ne = (25,  100)
goal_ne = (650, 500)


# In[5]:


# Static drone altitude (meters)
drone_altitude = 5
safety_distance = 2


# In[6]:

grid = create_grid(data, drone_altitude, safety_distance)
skeleton = medial_axis(invert(grid))


# Plot the edges on top of the grid along with start and goal locations.

# In[7]:


# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, cmap='Greys', origin='lower')
plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)
    
plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


# In[8]:


# TODO: Your start and goal location defined above
# will not necessarily be on the skeleton so you
# must first identify the nearest cell on the
# skeleton to start and goal
def find_start_goal(skel, start, goal):
    # TODO: find start and goal on skeleton
    # Some useful functions might be:
        # np.nonzero()
        # np.transpose()
        # np.linalg.norm()
        # np.argmin()
    non_zero = skel.nonzero()
    skel_cells = np.transpose(non_zero)
    near_start = None
    near_goal = None
    return near_start, near_goal

skel_start, skel_goal = find_start_goal(skeleton, start_ne, goal_ne)

print(start_ne, goal_ne)
print(skel_start, skel_goal)


# In[9]:

def heuristic_func(position, goal_position):
    # TODO: define a heuristic
    return None


# ### TODO: Run A* on the skeleton
# see [planning.py](/edit/planning.py) for a reminder on how to run the
# imported A* implementation (or rewrite it!)

# In[10]:


# Compare to regular A* on the grid
#path2, cost2 = a_star(grid, heuristic_func, start_ne, goal_ne)


# In[11]:

plt.imshow(grid, cmap='Greys', origin='lower')
plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)
# For the purposes of the visual the east coordinate lay along
# the x-axis and the north coordinates long the y-axis.
plt.plot(start_ne[1], start_ne[0], 'x')
# Uncomment the following as needed
#plt.plot(goal_ne[1], goal_ne[0], 'x')

#pp = np.array(path)
#plt.plot(pp[:, 1], pp[:, 0], 'g')
#pp2 = np.array(path2)
#plt.plot(pp2[:, 1], pp2[:, 0], 'r')
plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


# [solution](/notebooks/Medial-Axis-Solution.ipynb)
