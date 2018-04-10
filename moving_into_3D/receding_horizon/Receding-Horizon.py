
# coding: utf-8

# # Receding Horizon
#
# This notebook is your playground to pull together techniques from the
# previous lessons!  A solution here can be built from previous solutions (more
# or less) so we will offer no solution notebook this time.
#
# Here's a suggested approach:
#
# 1.  Load the colliders data
# 2.  Discretize your search space into a grid or graph
# 3.  Define a start and goal location
# 4.  Find a coarse 2D plan from start to goal
# 5.  Choose a location along that plan and discretize
#    a local volume around that location (for example, you
#    might try a 40x40 m area that is 10 m high discretized
#    into 1m^3 voxels)
# 6.  Define your goal in the local volume to a a node or voxel
#    at the edge of the volume in the direction of the next
#    waypoint in your coarse global plan.
# 7.  Plan a path through your 3D grid or graph to that node
#    or voxel at the edge of the local volume.
#
# We'll import some of the routines from previous exercises that you might find
# useful here.

# In[4]:

import numpy as np
import matplotlib.pyplot as plt

# Grid creation routine
from grid import create_grid
# Voxel map creation routine
from voxmap import create_voxmap
# 2D A* planning routine (can you convert to 3D??)
from planning import a_star
# Random sampling routine
from sampling import Sampler

#get_ipython().run_line_magic('matplotlib', 'inline')


# In[5]:

plt.rcParams['figure.figsize'] = 14, 14


# ## Load Data

# In[6]:


# This is the same obstacle data from the previous lesson.
filename = 'receding_horizon/colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)


# In[8]:

flight_altitude = 3
safety_distance = 3
grid = create_grid(data, flight_altitude, safety_distance)


# In[6]:

fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

plt.xlabel('NORTH')
plt.ylabel('EAST')

plt.show()

