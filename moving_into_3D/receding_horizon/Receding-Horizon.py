
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
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy.linalg as LA
from sklearn.neighbors import KDTree
from shapely.geometry import LineString
from queue import PriorityQueue

# Grid creation routine
from grid import create_grid
# Voxel map creation routine
from voxmap import create_voxmap
# 2D A* planning routine (can you convert to 3D??)
from planning import *
# Random sampling routine
from sampling import *

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

###################################
sampler = Sampler(data)
polygons = sampler.polygons
print(len(polygons))

xmin = np.min(data[:, 0] - data[:, 3])
xmax = np.max(data[:, 0] + data[:, 3])

ymin = np.min(data[:, 1] - data[:, 4])
ymax = np.max(data[:, 1] + data[:, 4])

zmin = 0
zmax = 10
print("X")
print("min = {0}, max = {1}\n".format(xmin, xmax))

print("Y")
print("min = {0}, max = {1}\n".format(ymin, ymax))

print("Z")
print("min = {0}, max = {1}".format(zmin, zmax))


# Next, it's time to sample points.  All that's left is picking the
# distribution and number of samples.  The uniform distribution makes sense in
# this situation since we we'd like to encourage searching the whole space.

# In[7]:
num_samples = 100

xvals = np.random.uniform(xmin, xmax, num_samples)
yvals = np.random.uniform(ymin, ymax, num_samples)
zvals = np.random.uniform(zmin, zmax, num_samples)

samples = np.array(list(zip(xvals, yvals, zvals)))


# In[8]:
samples[:10]


# ## Removing Points Colliding With Obstacles
#
# Prior to remove a point we must determine whether it collides with any
# obstacle.  Complete the `collides` function below.  It should return `True`
# if the point collides with *any* obstacle and `False` if no collision is
# detected.

# In[9]:
def collides(polygons, point):   
    # TODO: Determine whether the point collides
    # with any obstacles.
    for p in polygons:
        if p.polygon.contains(Point(point)) and p.height >= point[2]:
            return True
    return False


# Use `collides` for all points in the sample.

# In[10]:
t0 = time.time()
to_keep = []
for point in samples:
    if not collides(polygons, point):
        to_keep.append(point)
time_taken = time.time() - t0
print("Time taken {0} seconds ...".format(time_taken))


# In[11]:
print(len(to_keep))


# ## Points Visualization

# In[12]:
grid1 = create_grid(data, zmax, 3)


# In[13]:
fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

nmin = np.min(data[:, 0])
emin = np.min(data[:, 1])

# draw points
all_pts = np.array(to_keep)
north_vals = all_pts[:,0]
east_vals = all_pts[:,1]
plt.scatter(east_vals - emin, north_vals - nmin, c='red')

plt.ylabel('NORTH')
plt.xlabel('EAST')

plt.show()

###################################






voxmap = create_voxmap(data, 15)
print(voxmap.shape)


# Plot the 3D grid.

# In[6]:
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(voxmap, edgecolor='k')
ax.set_xlim(voxmap.shape[0], 0)
ax.set_ylim(0, voxmap.shape[1])
# add a bit to z-axis height for visualization
ax.set_zlim(0, voxmap.shape[2] + 20)

plt.xlabel('North')
plt.ylabel('East')

plt.show()






# In[30]:

nodes = sampler.sample(300)
print(len(nodes))
t0 = time.time()
g = create_graph(nodes, 10, polygons)
print('graph took {0} seconds to build'.format(time.time() - t0))



# In[31]:
print("Number of edges", len(g.edges))


# ## Step 4 - Visualize Graph

# In[13]:


#from grid import create_grid


# In[32]:
grid2 = create_grid(data, sampler._zmax, 3)


# In[33]:
fig = plt.figure()

plt.imshow(grid2, cmap='Greys', origin='lower')

nmin = np.min(data[:, 0])
emin = np.min(data[:, 1])

# draw edges
for (n1, n2) in g.edges:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black' , alpha=0.5)

# draw all nodes
for n1 in nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='blue')
    
# draw connected nodes
for n1 in g.nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
    


plt.xlabel('NORTH')
plt.ylabel('EAST')

plt.show()


# ## Step 5 - Define Heuristic

# In[34]:
def heuristic(n1, n2):
    # TODO: finish
    return LA.norm(np.array(n2) - np.array(n1))


# In[36]:
start = list(g.nodes)[0]
k = np.random.randint(len(g.nodes))
print(k, len(g.nodes))
goal = list(g.nodes)[k]


# In[37]:
path, cost = a_star_graph(g, heuristic, start, goal)
print(len(path), path)


# In[38]:
path_pairs = zip(path[:-1], path[1:])
for (n1, n2) in path_pairs:
    print(n1, n2)


# ## Step 7 - Visualize Path

# In[39]:
fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

nmin = np.min(data[:, 0])
emin = np.min(data[:, 1])

# draw nodes
for n1 in g.nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
    
# draw edges
for (n1, n2) in g.edges:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black')
    
# TODO: add code to visualize the path
path_pairs = zip(path[:-1], path[1:])
for (n1, n2) in path_pairs:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green')


plt.xlabel('NORTH')
plt.ylabel('EAST')

plt.show()