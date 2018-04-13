
# coding: utf-8

# # Probabilistic Roadmap
#
#
# In this notebook you'll expand on previous random sampling exercises by
# creating a graph from the points and running A*.
#
# 1.  Load the obstacle map data
# 2.  Sample nodes (use KDTrees here)
# 3.  Connect nodes (use KDTrees here)
# 4.  Visualize graph
# 5.  Define heuristic
# 6.  Define search method
# 7.  Execute and visualize
#
# We'll load the data for you and provide a template for visualization.

# In[1]:


# Again, ugly but we need the latest version of networkx!
# This sometimes fails for unknown reasons, please just
# "reset and clear output" from the "Kernel" menu above
# and try again!
import sys
get_ipython().system('{sys.executable} -m pip install -I networkx==2.1')
import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx


# In[2]:

nx.__version__ # should be 2.1


# In[3]:

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point, LineString
from queue import PriorityQueue

get_ipython().run_line_magic('matplotlib', 'inline')


# In[4]:

plt.rcParams['figure.figsize'] = 14, 14


# ## Step 1 - Load Data

# In[5]:


# This is the same obstacle data from the previous lesson.
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)


# ## Step 2 - Sample Points
#
#
# You may want to limit the z-axis values.

# In[6]:


# TODO: sample points randomly
# then use KDTree to find nearest neighbor polygon
# and test for collision


# ## Step 3 - Connect Nodes
#
# Now we have to connect the nodes.  There are many ways they might be done,
# it's completely up to you.  The only restriction being no edge connecting two
# nodes may pass through an obstacle.
#
# NOTE: You can use `LineString()` from the `shapely` library to create a line.
# Additionally, `shapely` geometry objects have a method `.crosses` which
# return `True` if the geometries cross paths, for instance your `LineString()`
# with an obstacle `Polygon()`!

# In[7]:


# TODO: connect nodes
# Suggested method
    # 1) cast nodes into a graph called "g" using networkx
    # 2) write a method "can_connect()" that:
        # casts two points as a shapely LineString() object
        # tests for collision with a shapely Polygon() object
        # returns True if connection is possible, False otherwise
    # 3) write a method "create_graph()" that:
        # defines a networkx graph as g = Graph()
        # defines a tree = KDTree(nodes)
        # test for connectivity between each node and
            # k of it's nearest neighbors
        # if nodes are connectable, add an edge to graph
    # Iterate through all candidate nodes!


# ## Step 4 - Visualize Graph

# In[8]:


# Create a grid map of the world
from grid import create_grid
# This will create a grid map at 1 m above ground level
grid = create_grid(data, 1, 1)

fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

nmin = np.min(data[:, 0])
emin = np.min(data[:, 1])

# If you have a graph called "g" these plots should work
# Draw edges
#for (n1, n2) in g.edges:
#    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin],
#    'black' , alpha=0.5)

# Draw all nodes connected or not in blue
#for n1 in nodes:
#    plt.scatter(n1[1] - emin, n1[0] - nmin, c='blue')
    
# Draw connected nodes in red
#for n1 in g.nodes:
#    plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
    

plt.xlabel('NORTH')
plt.ylabel('EAST')

plt.show()


# ## Step 5 - Define Heuristic

# In[10]:

def heuristic(n1, n2):
    # TODO: complete
    return 0


# ## Step 6 - Complete A*

# In[11]:

def a_star(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""
    
    # TODO: complete
    return []



# ## Step 7 - Visualize Path

# In[13]:

fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

# Add code to visualize path here
plt.xlabel('NORTH')
plt.ylabel('EAST')

plt.show()


# [solution](/notebooks/Probabilistic-Roadmap-Solution.ipynb)
